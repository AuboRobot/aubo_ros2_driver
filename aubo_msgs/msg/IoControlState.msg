builtin_interfaces/Time stamp

# Digital IO states. Array index matches the controller IO index.
bool[] standard_digital_inputs
bool[] standard_digital_outputs
bool[] configurable_digital_inputs
bool[] configurable_digital_outputs
bool[] tool_digital_inputs
bool[] tool_digital_outputs

# Analog IO states. Array index matches the controller IO index.
float64[] standard_analog_inputs
float64[] standard_analog_outputs
float64[] tool_analog_inputs
float64[] tool_analog_outputs

# Tool IO configuration states. Values map to the current AUBO SDK enum/domain values.
bool[] tool_io_inputs
int32[] tool_digital_input_actions
int32[] tool_digital_output_runstates
int32[] tool_analog_input_domains
int32[] tool_analog_output_domains
int32[] tool_analog_output_runstates
int32 tool_voltage_output_domain

bool tool_button_status
uint64 handle_status
int32 handle_device_state
