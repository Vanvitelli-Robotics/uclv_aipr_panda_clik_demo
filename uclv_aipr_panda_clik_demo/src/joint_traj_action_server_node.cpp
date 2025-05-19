#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "uclv_aipr_panda_clik_demo_interfaces/action/joint_traj.hpp"
#include "uclv_aipr_panda_clik_demo/quintic_trajectory.h"

namespace uclv
{

template <typename T>
std::string stdvect2string(const std::vector<T>& v)
{
  std::string s = "[";
  for (size_t i = 0; i < v.size(); i++)
  {
    s += std::to_string(v[i]);
    if (i < v.size() - 1)
      s += ", ";
  }
  s += "]";
  return s;
}

class JointTrajActionServer : public rclcpp::Node
{
  using JointTraj = uclv_aipr_panda_clik_demo_interfaces::action::JointTraj;
  using GoalHandleJointTraj = rclcpp_action::ServerGoalHandle<JointTraj>;

protected:
  rclcpp_action::Server<JointTraj>::SharedPtr action_server_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;

  /*
    Elenco dei giunti da muovere.
    In questo esempio semplificato assumeremo di leggere le q0 già ordinate
    nell'ordine che ci aspettiamo (dal giunto 1 al giunto 7)
  */
  // clang-format off
  std::vector<std::string> joint_names_ = {"panda_joint1",
                                           "panda_joint2",
                                           "panda_joint3",
                                           "panda_joint4",
                                           "panda_joint5",
                                           "panda_joint6",
                                           "panda_joint7"};
  // clang-format on

public:
  JointTrajActionServer(const rclcpp::NodeOptions opt = rclcpp::NodeOptions()) : Node("joint_traj_action_server", opt)
  {
    using namespace std::placeholders;

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 1, std::bind(&JointTrajActionServer::joint_state_cb, this, _1));

    joint_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("cmd/joint_position", 1);

    this->action_server_ = rclcpp_action::create_server<JointTraj>(
        this, "joint_traj_action", std::bind(&JointTrajActionServer::handle_goal, this, _1, _2),
        std::bind(&JointTrajActionServer::handle_cancel, this, _1),
        std::bind(&JointTrajActionServer::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "Joint Traj Action Server started");
  }

  /*
        Distruttore, non deve fare operazioni speciali.
    */
  ~JointTrajActionServer() = default;

protected:

  sensor_msgs::msg::JointState::ConstSharedPtr joint_states_;
  void joint_state_cb(const sensor_msgs::msg::JointState::ConstSharedPtr& msg)
  {
    joint_states_ = msg;
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const JointTraj::Goal> goal)
  {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Received goal request with\n duration: " << builtin_interfaces::msg::to_yaml(goal->duration)
                                                                 << "\n qf: " << stdvect2string(goal->qf));
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleJointTraj> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleJointTraj> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&JointTrajActionServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<GoalHandleJointTraj> goal_handle)
  {
    using namespace std::chrono_literals;

    auto goal = goal_handle->get_goal();

    /*
        Il seguente blocco serve per leggere la prima configurazione q0.

        Questa struttura è molto utile quando si vuole leggere un singolo nuovo messaggio.
        In generale la strategia è usare una variabile globale (joint_states_ in questo caso) che è accessibile sia dalla callback del subscriber
        che dal ciclo stesso.
    */
    // INIZIO BLOCCO PER LETTURA GIUNTI
    sensor_msgs::msg::JointState q0;
    joint_states_ = nullptr;
    while(!joint_states_)
    {
      std::this_thread::sleep_for(500ns);
    }
    q0 = *joint_states_;
    // FINE BLOCCO PER LETTURA GIUNTI

    // stampa per visualizzarla
    RCLCPP_INFO_STREAM(this->get_logger(), "\nqi is:\n" << sensor_msgs::msg::to_yaml(q0));

    /*
        Gestione del tempo ROS
        Nel ciclo mi servirà il tempo dall'inizio della traiettoria t.
        Uso this->now() che resistuisce il timestamp del PC (secondi trascorsi da 1/1/1970)
        Per avere t, mi salvo il tempo macchina dell'inizio della traiettoria t0.
        Poi t satà this->now()-t0
        Curiosità: gli oggetti Time e Duration in ROS hanno le definizioni degli operatori matematici
        TIME - TIME = DURATION
        http://wiki.ros.org/roscpp/Overview/Time
    */
    rclcpp::Time t0 = this->now();
    rclcpp::Duration t(0, 0);  // inizializzo t=0;

    // La nostra traiettoria sarà pubblicata con una certa frequenza.
    // Qui ho impostato 100 Hz
    rclcpp::Rate loop_rate(100.0);

    // Estraggo la duration in secondi
    double traj_duration = rclcpp::Duration(goal->duration).seconds();

    /*
        Vero e proprio ciclo di "generazione traiettoria"
        cicla finchè ros è ok e non ho finito la traiettoria
        t <= goal->duration significa t<tf.
    */
    while (rclcpp::ok() && t <= goal->duration)
    {
      // calcolo il nuovo t
      t = this->now() - t0;

      // check preemprtion dell'azione
      if (goal_handle->is_canceling())
      {
        auto result = std::make_shared<JointTraj::Result>();
        result->completed = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Joint traj Goal canceled");
        return;
      }

      // riempio il messaggio di comando
      sensor_msgs::msg::JointState cmd;
      // cmd.position è un std::vector. Di default ha size=0. Devo fare un resize!
      cmd.position.resize(joint_names_.size());
      // riempio il vettore usando la funzione quintic che ho creato nella libreria
      // "uclv_aipr_panda_clik_demo/quintic_trajectory.h" qui ho supposto che qi è già ordinato correttamente.
      for (int i = 0; i < 7; i++)
        cmd.position[i] = qintic(t.seconds(), q0.position[i], goal->qf[i], traj_duration);

      // è importante riempire il vettore dei nomi dei giunti.
      cmd.name = joint_names_;

      // riempio lo stamp, utile in caso di plot
      cmd.header.stamp = this->now();

      // ho definito come feedback il 'tempo che manca alla fine della traiettoria'
      // ovviamente non è l'unica scelta.
      auto feedbk_msg = std::make_shared<JointTraj::Feedback>();
      feedbk_msg->time_left = rclcpp::Duration(goal->duration) - t;

      // pubblico il feedback
      goal_handle->publish_feedback(feedbk_msg);

      // publico il comando in giunti
      joint_cmd_pub_->publish(cmd);

      // sleep sul rate
      loop_rate.sleep();
    }

    // se sono arrivato fin qui, non ci sono stati errori o preeption e la traiettoria è finita
    if (rclcpp::ok())
    {
      auto result = std::make_shared<JointTraj::Result>();
      result->completed = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Joint Traj Goal succeeded");
    }
  }
};

}  // namespace uclv

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uclv::JointTrajActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}