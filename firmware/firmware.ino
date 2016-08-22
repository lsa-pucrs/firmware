#include "Platypus.h"
#include "Components.h"
#include <adk.h>

// Arduino headers used in Platypus.h
// (informs the IDE to link these libraries)
#include <Servo.h>
#include <Scheduler.h>

// JSON parsing library
#include <ArduinoJson.h>

// TODO: remove me
#include "Board.h"

// ADK USB Host configuration 
char applicationName[] = "Platypus Server"; // the app on Android
char accessoryName[] = "Platypus Control Board"; // your Arduino board
char companyName[] = "Platypus LLC";
char versionNumber[] = "3.0";
char serialNumber[] = "3";
char url[] = "http://senseplatypus.com";

// ADK USB Host
USBHost Usb;
ADK adk(&Usb, companyName, applicationName, accessoryName, versionNumber, url, serialNumber);

// Android send/receive buffers
constexpr size_t INPUT_BUFFER_SIZE = 512;
char serial_buffer[INPUT_BUFFER_SIZE+1];

unsigned long last_command_time = 0;

const size_t OUTPUT_BUFFER_SIZE = 576;
char output_buffer[OUTPUT_BUFFER_SIZE+3];

// System state enumeration
enum SystemState
{
  /** There is no server currently communicating with the system. */
  DISCONNECTED,
  /** There is a server transmitting valid commands to the system. */
  CONNECTED,
};
SystemState system_state = DISCONNECTED;

// Time betweeen commands before we consider the Android
// server to be unresponsive.
const size_t COMMAND_TIMEOUT_MS = 1000;

// Define the systems on this board
// TODO: move this board.h?
platypus::Led rgb_led;

/**
 * Wrapper for ADK send command that copies data to debug port.
 * Requires a null-terminated char* pointer.
 */
void send(char *str) 
{
  // Add newline termination
  // TODO: Make sure we don't buffer overflow
  size_t len = strlen(str);
  str[len++] = '\r';
  str[len++] = '\n';
  str[len] = '\0';
  
  // Write string to USB.
  if (adk.isReady())
    adk.write(len, (uint8_t*)str);
  
  // Copy string to debugging console.
  //Serial.print("-> ");
  Serial.print(str);
}

/**
 * Returns a JSON error message to the connected USB device and the
 * serial debugging console.
 */
void reportError(const char *error_message, const char *buffer)
{
  // Construct a JSON error message.
  snprintf(output_buffer, OUTPUT_BUFFER_SIZE, 
           "{"
           "\"error\": \"%s\","
           "\"args\": \"%s\""
           "}",
           error_message, buffer);
  send(output_buffer);
}

/**
 * Handler to respond to incoming JSON commands and dispatch them to
 * configurable hardware components.
 */
void handleCommand(char *buffer)
{
  // Allocate buffer for JSON parsing
  StaticJsonBuffer<200> jsonBuffer;

  // Attempt to parse JSON in buffer
  JsonObject& command = jsonBuffer.parseObject(buffer);

  // Check for parsing error
  if (!command.success())
  {
    // Parsing Failure 
    reportError("Failed to parse JSON command.", buffer);
    return;
  }

  // If we have received a valid command, update system state.
  last_command_time = millis();

  for (JsonObject::iterator it=command.begin(); it!=command.end(); ++it)
  {
    const char * key = it->key;
    
    platypus::Configurable * target_object;
    size_t object_index;

    // Determine target object
    switch (key[0]){
    case 'm': // Motor command
      object_index = key[1] - '0';

      if (object_index >= board::NUM_MOTORS){
        reportError("Invalid motor index.", buffer);
        return;
      }

      target_object = platypus::motors[object_index];
      break;
      
    case 's': // Sensor command
      object_index = key[1] - '0';

      if (object_index >= board::NUM_SENSORS){
        reportError("Invalid sensor index.", buffer);
        return;
      }

      target_object = platypus::sensors[object_index];
      break;

    default: // Unrecognized target
      reportError("Unknown command target.", buffer);
      return;
    }

    // Extract JsonObject with param:value pairs
    JsonObject& params = it->value;

    // Todo: Move this parsing to specific components and pass ref to params instead
    // Iterate over and set parameter:value pairs on target object
    for (JsonObject::iterator paramIt=params.begin(); paramIt!=params.end(); ++paramIt)
    {
      const char * param_name = paramIt->key;
      const char * param_value = paramIt->value;

      /* Debugging Output
      Serial.print("Sending command to ");
      Serial.print(key);
      Serial.print(": ");
      Serial.print(param_name);
      Serial.print(" : ");
      Serial.println(param_value);
      */

      if (!target_object->set(param_name, param_value)) {
        reportError("Invalid parameter set.", buffer);
        continue; // Todo: Should we return or continue?
      }
    }
  }
}

void setup() 
{
  delay(1000);
  
  // Latch power shutdown line high to keep board from turning off.
  pinMode(board::PWR_KILL, OUTPUT);
  digitalWrite(board::PWR_KILL, HIGH);

  // Initialize debugging serial console.
  Serial.begin(115200);

  // Start the system in the disconnected state
  system_state = DISCONNECTED;
  
  // TODO: replace this with smart hooks.
  // Initialize sensors
  platypus::sensors[0] = new platypus::ServoSensor(0);
  platypus::sensors[1] = new platypus::GY26Compass(1);
  platypus::sensors[2] = new platypus::GY26Compass(2);
  platypus::sensors[3] = new platypus::GY26Compass(3);
  
  // Initialize motors
  platypus::motors[0] = new platypus::Dynamite(0);
  platypus::motors[1] = new platypus::Dynamite(1);

  // Power all peripherals
  platypus::motors[0]->enablePower(true);
  platypus::motors[1]->enablePower(true);

  // Make the ADK buffers into null terminated string.
  serial_buffer[INPUT_BUFFER_SIZE] = '\0';
  output_buffer[OUTPUT_BUFFER_SIZE] = '\0';

  // Set ADC Precision:
  analogReadResolution(12);
  
  // Create secondary tasks for system.
  Scheduler.startLoop(motorUpdateLoop);
  Scheduler.startLoop(serialCommandLoop);
  Scheduler.startLoop(usbCommandLoop);
  Scheduler.startLoop(batteryUpdateLoop);

  // Initialize Platypus library.
  platypus::init();
  
  // Print header indicating that board successfully initialized
  /*Serial.println(F("------------------------------"));
  Serial.println(companyName);
  Serial.println(url);
  Serial.println(accessoryName);
  Serial.println(versionNumber);
  Serial.println(F("------------------------------"));
  */
  // Turn LED off
  // TODO: Investigate how this gets turned on in the first place
  rgb_led.set(0, 0, 0);
  delay(1000);
}

/**
 * The main loop is _only_ used to clearly update the system state.
 */
void loop() 
{
  // Based on the command history, change the state of the system.
  if (millis() - last_command_time >= COMMAND_TIMEOUT_MS)
    system_state = DISCONNECTED;
  else
    system_state = CONNECTED;

  // Update the LEDs to match the system state.
  switch (system_state)
  {
  case DISCONNECTED:
    // Red blink
    rgb_led.set((millis() >> 8) & 1, 0, 0);
    break;
  case CONNECTED:
    // Green blink
    rgb_led.set(0, (millis() >> 8) & 1, 0);
    break;
  }
}

void usbCommandLoop()
{
  // Do USB bookkeeping.
  Usb.Task();
  
  // Wait for USB connection if we do not have it.
  if (!adk.isReady())
    return;
        
  // Allocate useful buffer variables.
  char usb_buffer[INPUT_BUFFER_SIZE+1];
  char * const buffer_start = usb_buffer;
  char * const buffer_end = usb_buffer + INPUT_BUFFER_SIZE;
  char *buffer_ptr = usb_buffer;
  uint32_t bytes_read = 0;

  // Loop forever while the connection is valid, reading in new characters.
  while (!adk.read(&bytes_read, buffer_end - buffer_ptr, (uint8_t *)buffer_ptr))
  {
    // Iterate through the new characters received, looking for the end of a message.
    for (uint32_t idx; idx < bytes_read; ++idx, ++buffer_ptr)
    {
      // If the buffer terminated, take everything up to this point and parse it.
      // Then, shift the remaining bytes to the beginning of the buffer and continue.
      if (*buffer_ptr == '\n')
      {
        *buffer_ptr = '\0';
        handleCommand(usb_buffer);
        memmove(usb_buffer, buffer_ptr, bytes_read - idx);
        buffer_ptr = usb_buffer;
      }
    }

    // If the buffer was exhausted, clear the entire thing.
    if (buffer_ptr == buffer_end)
      buffer_ptr = usb_buffer;

    // Defer to other threads while we wait for more input.
    Usb.Task();
    yield();
  }
}

void batteryUpdateLoop()
{
  int rawVoltage = analogRead(board::V_BATT);
  double voltageReading = 0.008879*rawVoltage + 0.09791;

  char output_str[128];
  snprintf(output_str, 128,
           "{"
           "\"s4\":{"
           "\"type\":\"battery\","
           "\"data\":\"%.3f %f %f\""
           "}"
           "}",
           voltageReading, 
           platypus::motors[0]->velocity(),
           platypus::motors[1]->velocity()
          );
  send(output_str);  
  delay(1000);
  yield();
}

/**
 * Periodically sends motor velocity updates.
 */
void motorUpdateLoop()
{
  // Wait for a fixed time period.
  delay(100);
  
  // Handle the motors appropriately for each system state.
  switch (system_state)
  {
  case DISCONNECTED:
    // Turn off motors.
    for (size_t motor_idx = 0; motor_idx < board::NUM_MOTORS; ++motor_idx) 
    {
      platypus::Motor* motor = platypus::motors[motor_idx];
      if (motor->enabled())
      {
        Serial.print("Disabling motor "); Serial.println(motor_idx);
        motor->disable();
      }
    }
    break;
  case CONNECTED:
    // Decay all motors exponentially towards zero speed.
    for (size_t motor_idx = 0; motor_idx < board::NUM_MOTORS; ++motor_idx) 
    {
      platypus::Motor* motor = platypus::motors[motor_idx];
      motor->set("v", "0.0");
    }

    // Rearm motors if necessary.
    for (size_t motor_idx = 0; motor_idx < board::NUM_MOTORS; ++motor_idx) 
    {
      platypus::Motor* motor = platypus::motors[motor_idx];
      if (!motor->enabled()) 
      {
        Serial.print("Arming motor "); Serial.print(motor_idx);
        motor->arm();
        Serial.println(F("Motor Armed"));
      }
    }
    break;
  }
  
  // Send status updates while connected to server.
  if (system_state == CONNECTED)
  {
    // TODO: move this to another location (e.g. Motor)
    // Send motor status update over USB
    snprintf(output_buffer, OUTPUT_BUFFER_SIZE,
      "{"
        "\"m0\":{"
          "\"v\":%f,"
          "\"c\":%f"
        "},"
        "\"m1\":{"
          "\"v\":%f,"
          "\"c\":%f"
        "}"
      "}",
      platypus::motors[0]->velocity(), platypus::motors[0]->current(),
      platypus::motors[1]->velocity(), platypus::motors[1]->current()
    );
    send(output_buffer);
  }
}

/**
 * Reads from serial debugging console and attempts to execute commands.
 */
void serialCommandLoop()
{
  // Index to last character in debug buffer.
  static size_t serial_buffer_idx = 0;
  
  // Wait until characters are received.
  while (!Serial.available()) yield();

  // Put the new character into the buffer, ignore \n
  char c = Serial.read();
  if (c != '\n')
    serial_buffer[serial_buffer_idx++] = c;
  
  // If it is the end of a line, or we are out of space, parse the buffer.
  if (serial_buffer_idx >= INPUT_BUFFER_SIZE || c == '\n')
  {
    // Properly null-terminate the buffer.
    serial_buffer[serial_buffer_idx] = '\0';
    serial_buffer_idx = 0;

    //Serial.println(debug_buffer);
    if (strcmp(serial_buffer, "DOc") == 0){
      platypus::sensors[1]->calibrate(1);
    } else if (strcmp(serial_buffer, "DOc0") == 0){
      platypus::sensors[1]->calibrate(0);
    } else if (strcmp(serial_buffer, "PHcm") == 0){
      platypus::sensors[2]->calibrate(0.0);
    } else if (strcmp(serial_buffer, "PHcl") == 0){
      platypus::sensors[2]->calibrate(-1);
    } else if (strcmp(serial_buffer, "PHch") == 0){
      platypus::sensors[2]->calibrate(1);
    }
    // Attempt to parse command.
    handleCommand(serial_buffer);
  }
}

