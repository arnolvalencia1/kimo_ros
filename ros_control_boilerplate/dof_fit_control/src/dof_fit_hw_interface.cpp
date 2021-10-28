

/* 
   Desc: Ejemplo de plantilla en blanco de interfaz de hardware ros_control para DOF_fit
             Para obtener un ejemplo de simulación más detallado, consulte sim_hw_interface.cpp
*/

#include <dof_fit_control/dof_fit_hw_interface.h>

namespace dof_fit_control
{
DOF_fitHWInterface::DOF_fitHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{
  ROS_INFO_NAMED("dof_fit_hw_interface", "DOF_fitHWInterface Ready.");
}

void DOF_fitHWInterface::read(ros::Duration& elapsed_time)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // AQUI DEBERIA IR EL CODIGO DE LECTURA
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void DOF_fitHWInterface::write(ros::Duration& elapsed_time)
{
  // La seguridad
  enforceLimits(elapsed_time);

  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // RELLENE SU COMANDO DE ESCRITURA A USB / ETHERNET / ETHERCAT / SERIAL ETC AQUÍ
  //
  // PARA UN EJEMPLO DE SIMULACIÓN FÁCIL O PARA CALCULAR EL CÓDIGO
  // VELOCIDAD DESDE POSICIÓN CON SUAVIZADO, VER
  // sim_hw_interface.cpp EN ESTE PAQUETE
  //
  // CÓDIGO DUMMY PASS-THROUGH
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
    joint_position_[joint_id] += joint_position_command_[joint_id];
  // FIN DEL CÓDIGO Ficticio
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void DOF_fitHWInterface::enforceLimits(ros::Duration& period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // ELIJA EL TIPO DE INTERFAZ DE LÍMITES DE UNIÓN QUE DESEA UTILIZAR
  // SÓLO DEBE USAR UNA INTERFAZ DE SATURACIÓN,
  // DEPENDIENDO DE SU MÉTODO DE CONTROL
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  // vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
  // vel_jnt_soft_limits_.enforceLimits(period);
  // eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

}  // namespace dof_fit_control
