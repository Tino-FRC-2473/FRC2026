---
config:
  layout: elk
---
stateDiagram
  direction TB
  [*] --> AI
  AI --> I:emergencyAbortBtnPressed
  AI --> AD1:nextBtn
  I --> I:!(nextBtn || manualOverrideBtnReleased || autoDownBtnPressed)
  I --> MDC:manualOverrideBtnPressed
  MDC --> MDC:!manualOverrideBtnReleased
  MDC --> I:manualOverrideBtnReleased
  
  %% Alignment
  I --> TA:nextBtn
  TA --> L1E:nextBtn && isAligned()

  %% L1 Sequence
  L1E --> L1E:!(nextBtn && MH_Latched())
  L1E --> L1R:nextBtn && MH_Latched()
  
  L1R --> L1R:!(nextBtn && SH_Latched())
  
  %% L2 Sequence
  L1R --> L2E:nextBtn && SH_Latched()
  L2E --> L2E:!(nextBtn && MH_Latched())
  L2E --> L2R:nextBtn && MH_Latched()
  
  L2R --> L2R:!(nextBtn && SH_Latched())
  
  %% L3 Sequence
  L2R --> L3E:nextBtn && SH_Latched()
  L3E --> L3E:!(nextBtn && MH_Latched())
  L3E --> LF:nextBtn && MH_Latched()

  %% Safety & Auto Down
  L1E --> I:emergencyAbortBtnPressed
  L1R --> I:emergencyAbortBtnPressed
  L2E --> I:emergencyAbortBtnPressed
  L2R --> I:emergencyAbortBtnPressed
  L3E --> I:emergencyAbortBtnPressed
  
  AD1 --> AD2:autoDownBtnPressed
  AD1 --> AD1:!autoDownBtnPressed
  AD2 --> I:isOnGround()
  AD2 --> AD2:!isOnGround()

  AI:AUTO_IDLE
  I:IDLE
  TA:TOWER_ALIGN
  MDC:MANUAL_DIRECT_CONTROL
  L1E:L1_EXTEND
  L1R:L1_RETRACT
  L2E:L2_EXTEND
  L2R:L2_RETRACT
  L3E:L3_EXTEND
  LF:LOCKED_FINAL
  AD1:AUTO_DOWN_ONE
  AD2:AUTO_DOWN_TWO