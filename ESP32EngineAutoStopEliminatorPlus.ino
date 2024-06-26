//
// Engine auto start-stop system eliminator firmware for SUBARU Levorg VN5
//
#include "driver/twai.h"
#include "Unit_4RELAY.h"
#include "subaru_levorg_vnx.h"

// Pins used to connect to 4-Relay Unit:
#define SDA_PIN GPIO_NUM_26
#define SCL_PIN GPIO_NUM_32

// Pins used to connect to CAN bus transceiver:
// #define RX_PIN GPIO_NUM_21
// #define TX_PIN GPIO_NUM_20
#define RX_PIN GPIO_NUM_19
#define TX_PIN GPIO_NUM_22

UNIT_4RELAY relay;

#define POLLING_RATE_MS 1000
static bool driver_installed = false;

enum debug_mode DebugMode = DEBUG;


uint16_t bytesToUint(uint8_t raw[], int shift, int size) {
  uint16_t result = 0;

  for (int i = 0; i < size; i++) {
    result = result << (sizeof raw[0] * 8);
    for (int j = 0; j < sizeof raw[0] * 8; j++) {
      result += raw[i + shift] & (1 << j);
    }
  }
  return result;
}


void print_frame(twai_message_t* twai_frame) {
  uint32_t CurrentTime;

  CurrentTime = micros();

  // Output all received message(s) to CDC port as candump -L
  if (twai_frame->rtr == 0) {  // Data Frame
    Serial.printf("(%d.%06d) can0 %03X#", CurrentTime / 1000000,
                  CurrentTime % 1000000,
                  twai_frame->identifier);
    for (uint8_t i = 0; i < twai_frame->data_length_code; i++) {
      Serial.printf("%02X", twai_frame->data[i]);
    }
    Serial.printf("\n");
  } else {  // Remote Frame
    Serial.printf("(%d.%06d) can0 %03X#R%d\n", CurrentTime / 1000000,
                  CurrentTime % 1000000,
                  twai_frame->identifier,
                  twai_frame->data_length_code);
  }
}


void send_cancel_frame(twai_message_t* rx_frame) {
  // Storage for transmit message buffer
  twai_message_t tx_frame;
  tx_frame.identifier = CAN_ID_CCU;
  tx_frame.data_length_code = 8;
  tx_frame.rtr = 0;
  tx_frame.extd = 0;
  tx_frame.ss = 1;
  tx_frame.self = 0;
  tx_frame.dlc_non_comp = 0;

  if ((rx_frame->data[1] & 0x0f) == 0x0f) {
    tx_frame.data[1] = rx_frame->data[1] & 0xf0;
  } else {
    tx_frame.data[1] = rx_frame->data[1] + 0x01;
  }
  tx_frame.data[2] = rx_frame->data[2];
  tx_frame.data[3] = rx_frame->data[3];
  tx_frame.data[4] = rx_frame->data[4];
  tx_frame.data[5] = rx_frame->data[5];
  tx_frame.data[6] = rx_frame->data[6] | 0x40;  // Eliminate engine auto stop bit on
  tx_frame.data[7] = rx_frame->data[7];
  // Calculate checksum
  tx_frame.data[0] = (tx_frame.data[1] + tx_frame.data[2] + tx_frame.data[3] + tx_frame.data[4] + tx_frame.data[5] + tx_frame.data[6] + tx_frame.data[7]) % SUM_CHECK_DIVIDER;
  if (twai_transmit(&tx_frame, pdMS_TO_TICKS(1000)) != ESP_OK) {
    if (DebugMode == DEBUG) {
      Serial.printf("# Error: Failed to queue message for transmission\n");
    }
  }
  if (DebugMode == DEBUG) {
    Serial.printf("# ");
    print_frame(&tx_frame);
  }
}


bool if_can_message_receive_is_pendig() {

  uint32_t alerts_triggered;
  twai_status_info_t twaistatus;

  // Check if alert happened
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));

  if (DebugMode == DEBUG) {
    twai_get_status_info(&twaistatus);
    // Handle alerts
    if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
      Serial.println("# Alert: TWAI controller has become error passive.");
    }
    if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
      Serial.println("# Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
      Serial.printf("# Bus error count: %d\n", twaistatus.bus_error_count);
    }
    if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      Serial.println("# Alert: The RX queue is full causing a received frame to be lost.");
      Serial.printf("# RX buffered: %d\t", twaistatus.msgs_to_rx);
      Serial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
      Serial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
    }
  }
  // If CAN message receive is pending, process the message
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    return true;
  } else {
    return false;
  }
}


void ViewOn() {
  relay.relayWrite(0, 1);
  delay(500);
  relay.relayWrite(0, 0);

  // Discard message(s) that received during delay()
  twai_clear_receive_queue();
}


void SModeOn() {
  relay.relayWrite(1, 1);
  delay(500);
  relay.relayWrite(1, 0);

  // Discard message(s) that received during delay()
  twai_clear_receive_queue();
}


void SModeOff() {
  relay.relayWrite(2, 1);
  delay(500);
  relay.relayWrite(2, 0);

  // Discard message(s) that received during delay()
  twai_clear_receive_queue();
}


void setup() {
  if (DebugMode != NORMAL) {
    Serial.begin(115200);
    while (!Serial)
      ;
  }

  // Initialize I2C
  relay.begin(&Wire, SDA_PIN, SCL_PIN);
  relay.Init(1);  // Set the lamp and relay to synchronous mode(Async = 0,Sync = 1).
  relay.relayAll(0);

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  //Look in the api-reference for other speed sets.
  // twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_filter_config_t f_config = { .acceptance_code = CAN_ID_CCU << 21, .acceptance_mask = (0x7fd << 21) | 0x1fffff, .single_filter = true };

  if (DebugMode == CANDUMP) {
    f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  }

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    if (DebugMode == DEBUG) {
      Serial.println("# Error: Failed to install driver");
    }
    return;
  }

  // Start TWAI driver
  if (twai_start() != ESP_OK) {
    if (DebugMode == DEBUG) {
      Serial.println("# Error: Failed to start driver");
    }
    return;
  }

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) != ESP_OK) {
    if (DebugMode == DEBUG) {
      Serial.println("# Error: Failed to reconfigure alerts");
    }
    return;
  }

  // TWAI driver is now successfully installed and started
  driver_installed = true;
}


void loop() {
  twai_message_t rx_frame;

  static enum cu_status TcuStatus = ENGINE_STOP;
  static enum cu_status CcuStatus = ENGINE_STOP;
  static enum status Status = PROCESSING;
  static uint16_t PreviousCanId = CAN_ID_CCU;
  static uint8_t Retry = 0;
  static uint8_t Shift = 0;
  static float AccelPos = 0;
  static float Speed = 0;
  static bool Over20kmh = false;
  static bool ShiftP = false;
  static bool SMode = false;
  static uint32_t SModeStart = 0;

  if (!driver_installed) {
    // Driver not installed
    delay(1000);
    return;
  }

  // If CAN message receive is pending, process the message
  if (if_can_message_receive_is_pendig()) {
    // One or more messages received. Handle all.
    while (twai_receive(&rx_frame, 0) == ESP_OK) {
      if (DebugMode == CANDUMP || (DebugMode == DEBUG && (rx_frame.identifier == CAN_ID_CCU || rx_frame.identifier == CAN_ID_TCU || rx_frame.identifier == CAN_ID_SCU || rx_frame.identifier == CAN_ID_MCU || rx_frame.identifier == CAN_ID_ECU))) {
        print_frame(&rx_frame);
      }

      if (rx_frame.rtr != 0 || rx_frame.data_length_code != 8) {
        continue;
      }

      if (DebugMode != CANDUMP) {
        switch (rx_frame.identifier) {
          case CAN_ID_ECU:
            AccelPos = bytesToUint(rx_frame.data, 4, 1) / 2.55;
            // if (DebugMode == DEBUG) {
            //   Serial.printf("# Information: AccelPos = %.1f \%).\n", AccelPos);
            // }
            if (ACCEL_THRESHOLD <= AccelPos) {
              if (!SMode) {
                // Change SI-Mode I -> S
                SModeOn();
                SMode = true;
                if (DebugMode == DEBUG) {
                  Serial.printf("# Information: Change I => S mode(Accel = %.1f \%).\n", AccelPos);
                }
              }
              SModeStart = millis();
            } else if (SMode) {
              if (S_MODE_TIME_LIMIT * 1000 < millis() - SModeStart) {  // over S_MODE_TIME_LIMIT sec
                // Change SI-Mode S -> I
                SModeOff();
                SMode = false;
                if (DebugMode == DEBUG) {
                  Serial.printf("# Information: Change S => I mode(%.1f sec).\n", (millis() - SModeStart) / 1000.0);
                  // Serial.printf("# millis(%d) - SModeStart(%d) = %d.\n", millis(), SModeStart, millis() - SModeStart);
                }
              }
            }
            PreviousCanId = rx_frame.identifier;
            break;

          case CAN_ID_MCU:
            if (Speed < 20 && 20 <= (rx_frame.data[2] + ((rx_frame.data[3] & 0x1f) << 8)) * 0.05625 && (!Over20kmh)) {
              Over20kmh = true;
              if (DebugMode == DEBUG) {
                Serial.printf("# Information:Auto View Off(%.1f km/h).\n", (rx_frame.data[2] + ((rx_frame.data[3] & 0x1f) << 8)) * 0.05625);
              }
            }

            if ((rx_frame.data[2] + ((rx_frame.data[3] & 0x1f) << 8)) * 0.05625 <= 15 && 15 < Speed && Over20kmh) {
              ViewOn();
              Over20kmh = false;
              if (DebugMode == DEBUG) {
                Serial.printf("# Information:View On(%.1f km/h).\n", (rx_frame.data[2] + ((rx_frame.data[3] & 0x1f) << 8)) * 0.05625);
              }
            }
            Speed = (rx_frame.data[2] + ((rx_frame.data[3] & 0x1f) << 8)) * 0.05625;
            PreviousCanId = rx_frame.identifier;
            break;

          case CAN_ID_SCU:
            if (Shift != (rx_frame.data[3] & 0x07)) {
              switch (rx_frame.data[3] & 0x07) {
                case P:
                  if (DebugMode == DEBUG) {
                    Serial.printf("# Information:Auto View Off(Shift 0x%x).\n", rx_frame.data[3] & 0x07);
                  }
                  // ParkingBrakeOn();
                  ShiftP = true;
                  break;
                case D:
                  if (DebugMode == DEBUG) {
                    Serial.printf("# Information: Change Another(0x%x) to D.\n", Shift);
                  }
                  if (ShiftP) {
                    ViewOn();
                    ShiftP = false;
                    if (DebugMode == DEBUG) {
                      Serial.printf("# Information:View On(Shift 0x%x).\n", rx_frame.data[3] & 0x07);
                    }
                  }
                  break;
                  // case R:
                  //   if (DebugMode == DEBUG) {
                  //     Serial.printf("# Information: Change Another to R.\n");
                  //   }
                  //   break;
                  // case N:
                  //   if (DebugMode == DEBUG) {
                  //     Serial.printf("# Information: Change Another to N.\n");
                  //   }
                  //   break;
              }
              Shift = rx_frame.data[3] & 0x07;
            }
            PreviousCanId = rx_frame.identifier;
            break;

          case CAN_ID_TCU:
            if ((rx_frame.data[2] & 0x08) != 0x08) {
              TcuStatus = NOT_READY;
            } else if (rx_frame.data[4] == 0xc0) {
              TcuStatus = IDLING_STOP_OFF;
              if (Retry != 0 && Status == PROCESSING) {
                if (DebugMode == DEBUG) {
                  // Output Information message
                  Serial.printf("# Information: Eliminate engine auto stop succeeded.\n");
                }
                Status = SUCCEEDED;
              }
            } else {
              TcuStatus = IDLING_STOP_ON;
              if (Status == SUCCEEDED) {
                if (DebugMode == DEBUG) {
                  // Output Information message
                  Serial.printf("# Information: Eliminate engine auto stop restarted.\n");
                }
                Status = PROCESSING;
                CcuStatus = NOT_READY;
                Retry = 0;
              }
            }
            PreviousCanId = rx_frame.identifier;
            break;

          case CAN_ID_CCU:
            if (PreviousCanId == CAN_ID_CCU) {  // TCU don't transmit message
              CcuStatus = ENGINE_STOP;
              TcuStatus = ENGINE_STOP;
              Status = PROCESSING;
              Retry = 0;
              Shift = 0;
              AccelPos = 0;
              Speed = 0;
              ShiftP = false;
              Over20kmh = false;
              SMode = false;
              SModeStart = 0;
              if (DebugMode == DEBUG) {
                // Output Information message
                Serial.printf("# Information: Engine stop.\n");
              }
            } else if (rx_frame.data[6] & 0x40) {
              if (DebugMode == DEBUG) {
                // Output Information message
                Serial.printf("# Information: Eliminate engine auto stop cancelled.\n");
              }
              Status = CANCELLED;
            } else if (Status == PROCESSING) {
              if (CcuStatus == NOT_READY || CcuStatus == ENGINE_STOP || TcuStatus == IDLING_STOP_OFF) {
                CcuStatus = READY;
              } else if (TcuStatus == IDLING_STOP_ON) {  // Transmit message for eliminate engine auto stop
                if (MAX_RETRY <= Retry) {                // Previous eliminate engine auto stop message failed
                  if (DebugMode == DEBUG) {
                    // Output Warning message
                    Serial.printf("# Warning: Eliminate engine auto stop failed\n");
                  }
                  Status = FAILED;
                } else {
                  Retry++;
                  // delay(50); // 50ms delay like real CCU
                  delay(50 / 2);
                  send_cancel_frame(&rx_frame);  // Transmit message
                  // Discard message(s) that received during delay()
                  twai_clear_receive_queue();
				          rx_frame.identifier = CAN_ID_TCU;
                  CcuStatus = NOT_READY;
                }
              } else {  // Unexpected case
                if (DebugMode == DEBUG) {
                  // Output Warning message
                  Serial.printf("# Warning: Unexpected case (CCU=%d TCU=%d).\n", CcuStatus, TcuStatus);
                }
              }
            }
            PreviousCanId = rx_frame.identifier;
            break;

            // default:  // Unexpected can id
            // if (DebugMode == DEBUG) {
            // Output Warning message
            // Serial.printf("# Warning: Unexpected can id (0x%03x).\n", rx_frame.identifier);
            // }
            // break;
        }
      }
    }
  }
}
