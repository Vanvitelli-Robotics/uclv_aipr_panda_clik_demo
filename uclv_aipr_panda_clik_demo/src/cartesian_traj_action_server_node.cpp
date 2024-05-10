#include <rclcpp/rclcpp.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "uclv_aipr_panda_clik_demo_interfaces/action/cartesian_traj.hpp"
#include "uclv_aipr_panda_clik_demo/quintic_trajectory.h"

namespace uclv
{

class CartesianTrajActionServer : public rclcpp::Node
{
  using CartesianTraj = uclv_aipr_panda_clik_demo_interfaces::action::CartesianTraj;
  using GoalHandleCartesianTraj = rclcpp_action::ServerGoalHandle<CartesianTraj>;

protected:
  rclcpp_action::Server<CartesianTraj>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cartesian_traj_pub_;

  // clang-format on

public:
  CartesianTrajActionServer(const rclcpp::NodeOptions opt = rclcpp::NodeOptions())
    : Node("cartesian_traj_action_server", opt)
  {
    using namespace std::placeholders;

    cartesian_traj_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("clik/desired_pose", 1);

    this->action_server_ = rclcpp_action::create_server<CartesianTraj>(
        this, "cartesian_traj_action", std::bind(&CartesianTrajActionServer::handle_goal, this, _1, _2),
        std::bind(&CartesianTrajActionServer::handle_cancel, this, _1),
        std::bind(&CartesianTrajActionServer::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "Cartesian Traj Action Server started");
  }

  /*
        Distruttore, non deve fare operazioni speciali.
    */
  ~CartesianTrajActionServer() = default;

protected:
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const CartesianTraj::Goal> goal)
  {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Received goal request with\n duration: " << builtin_interfaces::msg::to_yaml(goal->duration)
                                                                 << "\n p0: " << geometry_msgs::msg::to_yaml(goal->p0)
                                                                 << "\n pf: " << geometry_msgs::msg::to_yaml(goal->pf));
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCartesianTraj> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleCartesianTraj> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&CartesianTrajActionServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<GoalHandleCartesianTraj> goal_handle)
  {
    auto goal = goal_handle->get_goal();

    /*
     In questo generatore non ho bisogno della posa iniziale perchè mi viene data direttamente dal goal
    */
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
    // Qui ho impostato 1000 Hz
    rclcpp::Rate loop_rate(1000.0);

    // Estraggo la duration in secondi
    double traj_duration = rclcpp::Duration(goal->duration).seconds();

    /*
        Ciclo di "generazione traiettoria"
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
        auto result = std::make_shared<CartesianTraj::Result>();
        result->completed = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Cartesian traj Goal canceled");
        return;
      }

      // riempio il messaggio di comando usando la funzione quintic che ho creato nella libreria
      // "uclv_aipr_panda_clik_demo/quintic_trajectory.h"
      geometry_msgs::msg::PoseStamped out_msg;
      out_msg.pose.position.x = qintic(t.seconds(), goal->p0.position.x, goal->pf.position.x, traj_duration);
      out_msg.pose.position.y = qintic(t.seconds(), goal->p0.position.y, goal->pf.position.y, traj_duration);
      out_msg.pose.position.z = qintic(t.seconds(), goal->p0.position.z, goal->pf.position.z, traj_duration);
      // l'orientamento è costante e lo pongo uguale a pf.
      // Nota che, a causa dei limiti di questo generatore (genera solo una traiettoria in posizione e non in
      // orientamento) l'orientamento iniziale _DEVE_ essere lo stesso di qello finale. In questo esempio non faccio
      // nessun controllo ma sarebbe il caso di controllare che p0.orientation == pi.orientation e generare un eccezione
      // in caso negativo.
      out_msg.pose.orientation = goal->pf.orientation;

      // sto usando la versione Stamped del messaggio posa. Sarebbe opportuno riempire l'header.stamp (il tempo attuale
      // del messaggio)
      out_msg.header.stamp = this->now();

      // ho definito come feedback il 'tempo che manca alla fine della traiettoria'
      // ovviamente non è l'unica scelta.
      auto feedbk_msg = std::make_shared<CartesianTraj::Feedback>();
      feedbk_msg->time_left = rclcpp::Duration(goal->duration) - t;

      // pubblico il feedback
      goal_handle->publish_feedback(feedbk_msg);

      // publico il comando in cartesiano
      cartesian_traj_pub_->publish(out_msg);

      // sleep sul rate
      loop_rate.sleep();
    }

    // se sono arrivato fin qui, non ci sono stati errori o preeption e la traiettoria è finita
    if (rclcpp::ok())
    {
      auto result = std::make_shared<CartesianTraj::Result>();
      result->completed = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Cartesian Traj Goal succeeded");
    }
  }
};

}  // namespace uclv

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uclv::CartesianTrajActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}