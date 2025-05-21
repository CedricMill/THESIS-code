void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  // Called every time a Heartbeat message arrives from an ODrive
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  // Called every time feedback message arrives from an ODrive
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

void onCanMessage(const CanMsg& msg) {
  // Called for every message that arrives on the CAN bus
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }
}