# Climber FSM Spec

```mermaid
---
config:
  title: Climber State Diagram
---
stateDiagram

  [*] --> AUTO_IDLE

  AUTO_IDLE --> AUTO_DOWN_1: nextBtn
  AUTO_IDLE --> IDLE: emergencyAbortBtnPressed
  AUTO_IDLE --> AUTO_IDLE: !(nextBtn || emergencyAbortBtnPressed)

  IDLE --> MANUAL_DIRECT_CONTROL: manualOverrideBtnPressed
  IDLE --> L1_EXTEND: nextBtn
  IDLE --> AUTO_UP_1: autoUp1BtnPressed
  IDLE --> AUTO_UP_2: autoUp2BtnPressed
  IDLE --> IDLE: !(manualOverrideBtnPressed || nextBtn || autoUp1BtnPressed || autoUp2BtnPressed)

  MANUAL_DIRECT_CONTROL --> IDLE: nextBtn
  MANUAL_DIRECT_CONTROL --> MANUAL_DIRECT_CONTROL: !nextBtn

  L1_EXTEND --> IDLE: emergencyAbortBtnPressed
  L1_EXTEND --> L1_RETRACT: nextBtn && isExtendedL1()
  L1_EXTEND --> L1_EXTEND: !(emergencyAbortBtnPressed || (nextBtn && isExtendedL1()))

  L1_RETRACT --> IDLE: emergencyAbortBtnPressed
  L1_RETRACT --> LOCKED_FINAL: isRetractedL1()
  L1_RETRACT --> L1_RETRACT: !(emergencyAbortBtnPressed || isRetractedL1())

  LOCKED_FINAL --> LOCKED_FINAL: always

  AUTO_UP_1 --> AUTO_IDLE: isExtendedL1()
  AUTO_UP_1 --> AUTO_UP_1: !isExtendedL1()

  AUTO_UP_2 --> AUTO_IDLE: isRetractedL1()
  AUTO_UP_2 --> AUTO_UP_2: !isRetractedL1()

  AUTO_DOWN_1 --> AUTO_DOWN_2: nextBtn && isExtendedL1()
  AUTO_DOWN_1 --> AUTO_DOWN_1: !(nextBtn && isExtendedL1())

  AUTO_DOWN_2 --> IDLE: isOnGround()
  AUTO_DOWN_2 --> AUTO_DOWN_2: !isOnGround()

  IDLE: IDLE
  AUTO_IDLE: AUTO_IDLE
  MANUAL_DIRECT_CONTROL: MANUAL_DIRECT_CONTROL (Driver has control)
  L1_EXTEND: L1_EXTEND (Climber extends to L1 Height)
  L1_RETRACT: L1_RETRACT (Climber retracts to L1 Height)
  LOCKED_FINAL: LOCKED_FINAL (End of match, no transitions out)
  AUTO_UP_1: AUTO_UP_1 (Auto up part 1 - extend)
  AUTO_UP_2: AUTO_UP_2 (Auto up part 2 - retract)
  AUTO_DOWN_1: AUTO_DOWN_1 (Auto down part 1 - extend)
  AUTO_DOWN_2: AUTO_DOWN_2 (Auto down part 2 - reset to ground)