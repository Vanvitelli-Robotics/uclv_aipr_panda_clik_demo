#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_srvs/srv/set_bool.hpp>

#include <uclv_aipr_panda_clik_demo_interfaces/action/joint_traj.hpp>
#include <uclv_aipr_panda_clik_demo_interfaces/action/cartesian_traj.hpp>

// tf2
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  const std::string BASE_FRAME = "world";
  // La terna EE dovrebbe conicidere con quella settata nel nodo clik (EE_LINK_)
  const std::string EE_FRAME = "panda_hand";
  //   const std::string EE_FRAME = "panda_hand_tcp";  //<-- this end effector is not present in the urdf, to use it you
  //   need to download the modified
  // package: https://github.com/Vanvitelli-Robotics/moveit_resources_panda_moveit_config

  auto node = std::make_shared<rclcpp::Node>("task_node");

  // Inizializzo gli action client di traiettoria cartesiana e in giunti
  auto joint_traj_action_client =
      rclcpp_action::create_client<uclv_aipr_panda_clik_demo_interfaces::action::JointTraj>(node, "joint_traj_action");
  auto cartesian_traj_action_client =
      rclcpp_action::create_client<uclv_aipr_panda_clik_demo_interfaces::action::CartesianTraj>(node,
                                                                                                // clang-format off
                                                                                                "cartesian_traj_action"
                                                                                                // clang-format on
      );

  // service client per attivare il clik
  auto set_run_clik_client = node->create_client<std_srvs::srv::SetBool>("clik/set_run");

  // Aspetta che i Server siano UP
  RCLCPP_INFO(node->get_logger(), "Waiting for servers...");
  joint_traj_action_client->wait_for_action_server();
  cartesian_traj_action_client->wait_for_action_server();
  set_run_clik_client->wait_for_service();
  RCLCPP_INFO(node->get_logger(), "Servers UP");

  // piccola variabile di supporto per fermare la demo nei vari passi
  char ans;

  // Traiettoria spazio giunti
  {
    // Attesa dell'utente
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Traiettoria spazio giunti... - Iserisci un carattere qualsiasi e premi invio:");
    std::cin >> ans;

    // Definizione della q_finale, il valore è scelto per portare il robot in una configurazione "destra" non singolare
    uclv_aipr_panda_clik_demo_interfaces::action::JointTraj::Goal joint_goal;
    joint_goal.duration = rclcpp::Duration::from_seconds(3.0);
    joint_goal.qf = { 0.0, 0.0, 0.0, -3.0 / 4.0 * M_PI, 0.0, 3.0 / 4.0 * M_PI, M_PI_4 };

    // chiamo l'azione e aspetto che termini
    auto future_goal_handle = joint_traj_action_client->async_send_goal(joint_goal);
    if (rclcpp::spin_until_future_complete(node, future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, GOAL NOT SENT");
      return -1;
    }
    auto future_result = joint_traj_action_client->async_get_result(future_goal_handle.get());
    if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, NO RESULT");
      return -1;
    }
    // se arrivo quì l'azione è terminata, controllo se è terminata con successo

    // check dello stato dell'azione, se non ho errori lo stato deve essere SUCCEEDED
    if (future_result.get().code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, JOINT TRAJECTORY NOT SUCCEEDED");
      return -1;
    }
    if (!future_result.get().result->completed)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, JOINT TRAJECTORY NOT COMPLETED");
      return -1;
    }

    // Attesa dell'utente
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Traiettoria spazio giunti... OK - Iserisci un carattere qualsiasi e premi invio:");
    std::cin >> ans;
  }

  // attivazione CLIK
  {
    // Attesa dell'utente
    RCLCPP_INFO_STREAM(node->get_logger(), "Attivo il CLIK... - Iserisci un carattere qualsiasi e premi invio:");
    std::cin >> ans;

    // Attivo il CLIK chiamando il servizio di attivazione
    auto set_bool_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    set_bool_request->data = true;  // <-- true = attiva clik

    // chiamo il servizio e aspetto la risposta
    auto future_result = set_run_clik_client->async_send_request(set_bool_request);
    if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
      return -1;
    }
    // se arrivo quì il servizio è terminato, controllo se è terminato con successo

    // check del successo del servizio
    if (!future_result.get()->success)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "ERROR, SERVICE NOT SUCCESSFUL... message: " << future_result.get()->message);
      return -1;
    }

    // Attesa dell'utente
    RCLCPP_INFO_STREAM(node->get_logger(), "Attivo il CLIK... OK - Iserisci un carattere qualsiasi e premi invio:");
    std::cin >> ans;
  }

  // goal cartesiano, lo dichiaro prima perchè lo uso per conservare p0 e pf
  uclv_aipr_panda_clik_demo_interfaces::action::CartesianTraj::Goal cartesian_goal;

  // lettura pi (posa iniziale)
  {
    // Attesa dell'utente
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Leggo la posa iniziale... - Iserisci un carattere qualsiasi e premi "
                       "invio:");
    std::cin >> ans;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
      using namespace std::chrono_literals;
      transformStamped = tf_buffer->lookupTransform(BASE_FRAME, EE_FRAME, tf2::TimePointZero, 5s);
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "TF ERROR " << ex.what());
      return -1;
    }

    // Copio la posa iniziale nel goal del Cartesian Traj
    cartesian_goal.p0.position.x = transformStamped.transform.translation.x;
    cartesian_goal.p0.position.y = transformStamped.transform.translation.y;
    cartesian_goal.p0.position.z = transformStamped.transform.translation.z;
    cartesian_goal.p0.orientation = transformStamped.transform.rotation;

    RCLCPP_INFO_STREAM(node->get_logger(), "La posa iniziale e' \n" << geometry_msgs::msg::to_yaml(cartesian_goal.p0));

    // Attesa dell'utente
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Leggo la posa iniziale... OK - Iserisci un carattere qualsiasi e premi invio:");
    std::cin >> ans;
  }

  // scelta di pf (posa finale)
  {
    // Attesa dell'utente
    RCLCPP_INFO_STREAM(node->get_logger(), "Setto la posa finale... - Iserisci un carattere qualsiasi e premi invio:");
    std::cin >> ans;

    // La posa finale la scelgo a partire dalla posa iniziale dando un delta di 10cm lungo x e z, e 30cm lungo y.
    cartesian_goal.pf = cartesian_goal.p0;
    cartesian_goal.pf.position.z += 0.1;
    cartesian_goal.pf.position.x -= 0.1;
    cartesian_goal.pf.position.y += 0.3;

    RCLCPP_INFO_STREAM(node->get_logger(), "La posa finale e' \n" << geometry_msgs::msg::to_yaml(cartesian_goal.pf));

    // Attesa dell'utente
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Setto la posa finale... OK - Iserisci un carattere qualsiasi e premi invio:");
    std::cin >> ans;
  }

  // Movimento cartesiano
  {
    // Attesa dell'utente
    RCLCPP_INFO_STREAM(node->get_logger(), "Move Cartesian... - Iserisci un carattere qualsiasi e premi invio:");
    std::cin >> ans;

    cartesian_goal.duration = rclcpp::Duration::from_seconds(3.0);

    // chiamo l'azione e aspetto che termini
    auto future_goal_handle = cartesian_traj_action_client->async_send_goal(cartesian_goal);
    if (rclcpp::spin_until_future_complete(node, future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, GOAL NOT SENT");
      return -1;
    }
    auto future_result = cartesian_traj_action_client->async_get_result(future_goal_handle.get());
    if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, NO RESULT");
      return -1;
    }
    // se arrivo quì l'azione è terminata, controllo se è terminata con successo

    // check dello stato dell'azione, se non ho errori lo stato deve essere SUCCEEDED
    if (future_result.get().code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, CARTESIAN TRAJECTORY NOT SUCCEEDED");
      return -1;
    }
    if (!future_result.get().result->completed)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, CARTESIAN TRAJECTORY NOT COMPLETED");
      return -1;
    }

    // Attesa dell'utente
    RCLCPP_INFO_STREAM(node->get_logger(), "Move Cartesian... OK - Iserisci un carattere qualsiasi e premi invio:");
    std::cin >> ans;
  }

  // Stop del clik
  {
    // Attesa dell'utente
    RCLCPP_INFO_STREAM(node->get_logger(), "STOP del CLIK... - Iserisci un carattere qualsiasi e premi invio:");
    std::cin >> ans;

    // Stoppo il CLIK chiamando il servizio di attivazione/disattivazione
    auto set_bool_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    set_bool_request->data = false;  // <-- false = disattiva clik

    // chiamo il servizio e aspetto la risposta
    auto future_result = set_run_clik_client->async_send_request(set_bool_request);
    if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
      return -1;
    }
    // se arrivo quì il servizio è terminato, controllo se è terminato con successo

    // check del successo del servizio
    if (!future_result.get()->success)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "ERROR, SERVICE NOT SUCCESSFUL... message: " << future_result.get()->message);
      return -1;
    }

    // Attesa dell'utente
    RCLCPP_INFO_STREAM(node->get_logger(), "STOP del CLIK... OK - Iserisci un carattere qualsiasi e premi invio:");
    std::cin >> ans;
  }

  // FINE
  {
    // Attesa dell'utente
    RCLCPP_INFO_STREAM(node->get_logger(), "FINE - Iserisci un carattere qualsiasi e premi invio:");
    std::cin >> ans;
  }
  rclcpp::shutdown();
  return 0;
}
