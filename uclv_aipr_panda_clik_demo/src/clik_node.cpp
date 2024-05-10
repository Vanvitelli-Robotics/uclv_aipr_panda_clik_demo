#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <Eigen/Dense>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace uclv
{

/*
  Funzione che assicura la continuità del quaternione.
  Dato un quaternione Q, si ha che -Q rappresenta la stessa rotazione.
  Quando si trasforma una matrice di rotazione in un Quaternione potrei avere indistintamente +Q o -Q.
  Questo crea un problema nell'inversione cinemarica che si può ritrovare un errore discontinuo.
  Questa funzione prende il quaternione q (al passo corrente) e prende il quaternione oldQ (al passo precendete)
  e restituisce un quaternione "equivalente a q" (rappresenta la stessa rotazione) ma "continuo" rispetto ad oldQ

  Non ci interessano i dettagli implementativi di questa funzione, è stata data nel pdf.
*/
Eigen::Quaterniond quaternionContinuity(const Eigen::Quaterniond& q, const Eigen::Quaterniond& oldQ)
{
  auto tmp = q.vec().transpose() * oldQ.vec();
  if (tmp < -0.01)
  {
    Eigen::Quaterniond out(q);
    out.vec() = -out.vec();
    out.w() = -out.w();
    return out;
  }
  return q;
}

class CLIKNode : public rclcpp::Node
{
protected:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_run_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;

  rclcpp::TimerBase::SharedPtr clik_timer_;

  //#####################################
  /*
      Qualche variabile utile
  */

  // Variabili di controllo
  double Ts_ = 0.001;             // <-- periodo di campionamento del clik
  double clik_gain_ = 1.0 / Ts_;  // <-- guadagno del clik

  std::string ROBOT_MODEL_GROUP = "panda_arm";
  std::string EE_LINK_ = "panda_hand";
  // std::string EE_LINK_ = "panda_hand_tcp"; //<-- this end effector is not present in the urdf, to use it you need to
  // download the modified package: https://github.com/Vanvitelli-Robotics/moveit_resources_panda_moveit_config

  // il RobotModelLoader serve per caricare il modello da URDF con la libreria MoveIt!
  // è usato per costruire l'oggetto RobotState kinematic_state_
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  rclcpp::Node::SharedPtr robot_loader_node_;

  const moveit::core::JointModelGroup* joint_model_group_;
  const moveit::core::LinkModel* last_link_;

  // lo stato cinematico del robot, possiamo leggerlo come lo stato
  // dell'algoritmo CLIK (le q - variabili di giunto)
  moveit::core::RobotStatePtr kinematic_state_;

  // oldQ usato per la continuità del quaternione, Inizializzato all'identità.
  Eigen::Quaterniond oldQuaternion_ = Eigen::Quaterniond::Identity();

  Eigen::Vector3d reference_point_position_ = Eigen::Vector3d::Zero();  // <-- mi servirà per calcolare lo Jacobiano

  //#####################################

public:
  CLIKNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions()) : Node("clik", opt)
  {
    using namespace std::placeholders;

    // #############################################################################################################
    /*
       Alcune variabili che mi servono per usare la libreria moveit (RobotState)
       - costruisco il kinematic_model
       - costruisco la variabile globale kinematic_state
       - definisco il 'last_link', il link da usare come end effector. In questo esempio scelgo l'ultimo link del gruppo
       panda_arm
   */
    robot_loader_node_ = std::make_shared<rclcpp::Node>(
        "robot_model_loader", rclcpp::NodeOptions(opt).automatically_declare_parameters_from_overrides(true));
    robot_model_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(robot_loader_node_);
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader_->getModel();
    RCLCPP_INFO(this->get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str());
    joint_model_group_ = kinematic_model->getJointModelGroup(ROBOT_MODEL_GROUP);
    kinematic_state_ = moveit::core::RobotStatePtr(new moveit::core::RobotState(kinematic_model));
    last_link_ = kinematic_state_->getLinkModel(EE_LINK_);

    // #############################################################################################################
    // Inizializzo i subscriber
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "clik/desired_pose", 1, std::bind(&CLIKNode::pose_desired_cb, this, _1));
    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "clik/desired_twist", 1,
        std::bind(&CLIKNode::twist_desired_cb, this, _1));  // ricorda che in questo esempio non riceviamo veramente
                                                            // il twist desiderato

    // Il publisher per i comandi in giunto
    cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("cmd/joint_position", 1);

    // Inizializzo il servizio di set_run del clik
    srv_set_run_ =
        this->create_service<std_srvs::srv::SetBool>("clik/set_run", std::bind(&CLIKNode::set_run_cb, this, _1, _2));
    // #############################################################################################################

    // timer of the main control loop
    // uso un timer che garantisce la giusto frequenza al clik
    rclcpp::Duration period = rclcpp::Duration::from_seconds(Ts_);
    clik_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(Ts_ / 1000.0)),
                                          std::bind(&CLIKNode::clik_control_cycle, this));
    clik_timer_->cancel();  // <-- il timer è disattivato all'inizio
    // b_clik_run_ = false;

    RCLCPP_INFO(this->get_logger(), "CLIK Node started");
  }

  ~CLIKNode() = default;

protected:
  //############################################################################################################
  /*
    Callbk per la lettura della posa desiderata
    Nota l'utilizzo degli attributi dell'oggetto per "comunicare" con il ciclo pricipale la posa desiderata.
*/
  Eigen::Vector3d position_des_;
  Eigen::Quaterniond quaternion_des_;
  void pose_desired_cb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
  {
    position_des_(0) = msg->pose.position.x;
    position_des_(1) = msg->pose.position.y;
    position_des_(2) = msg->pose.position.z;

    quaternion_des_.x() = msg->pose.orientation.x;
    quaternion_des_.y() = msg->pose.orientation.y;
    quaternion_des_.z() = msg->pose.orientation.z;
    quaternion_des_.w() = msg->pose.orientation.w;

    // assicuro che il quaternione che ricevo come desiderato è "continuo" con lo stato attuale del robot nel nodo clik
    quaternion_des_ = quaternionContinuity(quaternion_des_, oldQuaternion_);
  }

  /*
    Callbk per la lettura della velocità (Twist) desiderato
    Nota l'utilizzo degli attributi dell'oggetto per "comunicare" con il ciclo pricipale la posa desiderata.
    In generatore di traiettoria potrebbe restituire anche la velocità, in questo caso il CLIK potrebbe beneficiare
    della conoscenza di x_dot (la velocità della traiettoria) per implementare l'azione in feedforward.
    In questo esempio il generatore non pubblica la velocità e vel_des_=ZERO sempre.
    L'algoritmo CLIK funziona anche così, ma potrebbe essere interessante implementare nel generatore di traiettoria
    la pubblicazione anche del Twist.
*/
  Eigen::Matrix<double, 6, 1> vel_des_;
  void twist_desired_cb(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg)
  {
    vel_des_(0) = msg->twist.linear.x;
    vel_des_(1) = msg->twist.linear.y;
    vel_des_(2) = msg->twist.linear.z;
    vel_des_(3) = msg->twist.angular.x;
    vel_des_(4) = msg->twist.angular.y;
    vel_des_(5) = msg->twist.angular.z;
  }
  //############################################################################################################

  //#####################################
  /*
      Callbk del servizio di attivazione del CLIK
      Il nodo di default non fa nulla (b_clik_run=false)
      Questo servizio permette di attivare il nodo (setta b_clik_run=true) e contestualmente lo inizializza

      *LA FASE DI INIZIALIZZAZIONE E' IMPORTANTE*
      quando viene richiesto di settare b_clik_run=True bisogna:
      - Settare lo stato del clik (memorizzato in kinematic_state sopra) ai valori correnti del robot vero.
      - Settare la posa desiderata (position_des, quaternion_des) al valore corrente di cinematica diretta (in modo da
     avere un errore iniziale nullo)
      - Settare il valore di azione in avanti (vel_des_) a zero
      Il motivo per cui inizializziamo anche gli "input" del CLIK (position_des, quaternion_des,vel_des_) è perchè
     l'arrivo del messaggio di comando non è istantaneo, tra l'attivazione del CLIK e l'arrivo del primo comando il
     robot non deve muoversi (l'errore del clik è zero)
  */
  void set_run_cb(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res)
  {
    using namespace std::chrono_literals;

    // Sfrutto il servizio standard SetBool per ricevere la richiesta si start (true) o stop (false)

    if (!req->data)  // <- se ho rishiesto stop
    {
      // semplicemente setto b_clik_run a false e ritorno.
      clik_timer_->cancel();  // <-- disattivo il timer = stop del clik
      // b_clik_run_ = false;
      RCLCPP_INFO_STREAM(this->get_logger(), "CLIK STOP");
      res->message = "stop";
      res->success = true;
      return;
    }

    // se sono qui è stato richiesto uno start.

    // Devo leggere i valori correnti delle variabili di giunto per inizializzare il clik
    // in questo esempio uso rclcpp::wait_for_message per leggere un singolo messaggio dal topic /joint_states
    RCLCPP_INFO_STREAM(this->get_logger(), "CLIK waiting joint state...");
    sensor_msgs::msg::JointState joint_current;
    if (!rclcpp::wait_for_message<sensor_msgs::msg::JointState>(joint_current, shared_from_this(), "joint_states", 10s))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get current joint state");
      // b_clik_run_ = false;
      RCLCPP_INFO_STREAM(this->get_logger(), "CLIK STOP!");
      res->message = "stop";
      res->success = false;
      return;
    }

    // aggiorno la variabile globale kinematic_state con i valori letti delle variabili di giunto
    for (size_t i = 0; i < joint_current.name.size(); ++i)
    {
      kinematic_state_->setJointPositions(joint_current.name[i], &joint_current.position[i]);
    }

    //####################
    // Qusesto blocco esegue la cinematica diretta (kinematic_state_->getGlobalLinkTransform) e inizializza i valori
    // desiderati in modo da avere errore iniziale nullo Le prime due righe servono per ottenere il link rispetto al
    // quale fare la cinematica diretta (rivedi il tutorial di MoveIt sul RobotState)
    const Eigen::Isometry3d& b_T_e = kinematic_state_->getGlobalLinkTransform(last_link_);
    position_des_ = b_T_e.translation();
    quaternion_des_ = b_T_e.rotation();
    oldQuaternion_ = quaternion_des_;
    vel_des_.setZero();
    //####################

    //#####################
    // Piccolo check di debug. Stampo le variabili di giunto appena salvate in kinematic_state.
    Eigen::VectorXd q;
    kinematic_state_->copyJointGroupPositions(joint_model_group_, q);
    RCLCPP_INFO_STREAM(this->get_logger(), "CLIK start! - q:\n" << q);
    RCLCPP_INFO_STREAM(this->get_logger(), "position_des:\n" << position_des_);
    RCLCPP_INFO_STREAM(this->get_logger(), "quaternion_des:\n"
                                               << quaternion_des_.w() << " - " << quaternion_des_.vec().transpose());

    for (size_t i = 0; i < joint_model_group_->getActiveJointModelNames().size(); i++)
      RCLCPP_INFO_STREAM(this->get_logger(), "getActiveJointModelNames:["
                                                 << i << "]:" << joint_model_group_->getActiveJointModelNames()[i]);
    //#####################

    // finalmente setto b_clik_run a true e ritorno il servizio
    // b_clik_run_ = true;
    clik_timer_->reset();  // <-- attivo il timer = start del clik

    res->success = true;
    res->message = "start";
  }

  //#####################################
  // Ciclo principale del nodo
  void clik_control_cycle()
  {
    //###########################################################
    // Calcolo l'errore cinematico

    // Cinematica diretta
    const Eigen::Isometry3d& b_T_e = kinematic_state_->getGlobalLinkTransform(last_link_);

    // Estraggo la posizione
    Eigen::Vector3d position = b_T_e.translation();
    // Estraggo il quaternione
    Eigen::Matrix3d rotation = b_T_e.rotation();
    Eigen::Quaterniond quaternion(rotation);
    // Assicuro la continuità del quaternione
    quaternion = quaternionContinuity(quaternion, oldQuaternion_);
    oldQuaternion_ = quaternion;  // <-- per il prissimo ciclo

    // Calcolo l'errore 6D
    // - userò il metodo block delle matrici Eigen, potrei accedere anche ai
    //   singoli elementi del vettore usando qualche riga di codice in più
    Eigen::Matrix<double, 6, 1> error;

    // errore in posizione (setto il blocco di dimensioni 3x1 che parte dalla posizione (0,0))
    error.block<3, 1>(0, 0) = position_des_ - position;
    // Errore in orientamento è la parte vettoriale di Qd*inv(Q)
    Eigen::Quaterniond deltaQ = quaternion_des_ * quaternion.inverse();
    // errore in orientamento (setto il blocco di dimensioni 3x1 che parte dalla posizione (3,0))
    error.block<3, 1>(3, 0) = deltaQ.vec();
    //###########################################################

    //###########################################################
    // Calcolo le velocità di giunto con la seguente formula
    // qdot = Jpinv * (v_des + gamma*error);
    //  - Jpinv è la pseudoinversa dello Jacobiano
    //  - v_des è la velocità della traiettoria (azione in avanti, è zero in questo esempio...)
    //  - error è l'errore calcolato al passo precendete e gamma è il guadagno

    // vel_e è il termine (v_des + gamma*error)
    Eigen::Matrix<double, 6, 1> vel_e = vel_des_ + clik_gain_ * error;

    // Calcolo lo jacobiano
    Eigen::MatrixXd jacobian;
    kinematic_state_->getJacobian(joint_model_group_, last_link_, reference_point_position_, jacobian);

    // Calcolo pinv(jacobian)*vel_e, ovvero q_dot
    Eigen::VectorXd q_dot = jacobian.completeOrthogonalDecomposition().solve(vel_e);
    //###########################################################

    //###########################################################
    // Devo passare da q_dot a q -----> devo integrare.
    // Utilizzo una integrazione alla Eulero: q(k+1) = q(k) + T*q_dot(k)

    // copio i valori attuali del kinematic state (q(k))
    Eigen::VectorXd q;
    kinematic_state_->copyJointGroupPositions(joint_model_group_, q);

    // aggiorno q con la formula di Eulero (ottengo q(k+1))
    q = q + q_dot * Ts_;  // ATTENZIONE: Ts_ è il periodo di campionamento del clik, DEVE essere lo stesso del timer

    // copio i nuovi valori di q nell'oggetto kinematic_state
    kinematic_state_->setJointGroupPositions(joint_model_group_, q);
    //###########################################################

    //###########################################################
    // Finalmente pubblico il risultato

    // definisco i joint state di comando copiando le position dal kinematic state
    sensor_msgs::msg::JointState out_msg;
    kinematic_state_->copyJointGroupPositions(joint_model_group_, out_msg.position);
    // infine copio i nomi dei giunti dal joint_model_group (PASSAGGIO FONDAMENTALE)
    out_msg.name = joint_model_group_->getActiveJointModelNames();

    // riempio lo stamp, utile in caso di plot
    out_msg.header.stamp = this->now();

    cmd_pub_->publish(out_msg);
    //###########################################################
  }
};

}  // namespace uclv

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uclv::CLIKNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}