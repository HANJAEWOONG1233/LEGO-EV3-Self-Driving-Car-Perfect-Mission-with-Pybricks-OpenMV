╔══════════════════════════════════════════════════╗  
║      🚀  LEGO EV3 Self-Driving Car Mission       ║  
╚══════════════════════════════════════════════════╝  

✨ **Powered by** Pybricks   🐍 **Language:** MicroPython  

────────────────────────────────────────────────────  
🎯 **Project Highlights**  
• **State-Machine Control**  
  Isolates each mission phase for rock-solid logic flow.  
• **Advanced Vision**  
  OpenMV camera via UART recognizes traffic lights in real time.  
• **Custom P-Control**  
  Balances left/right sensor groups for ultra-stable line tracing.  
• **Multi-Stage Timing**  
  Non-blocking boosts, cruises & waits orchestrated by StopWatch.  

────────────────────────────────────────────────────  
🎥 **Watch it in Action**  
▶️ https://youtu.be/F8zVAMhfwYk  

────────────────────────────────────────────────────  
📜 **Mission Sequence**  

1. **Start**  
   ▸ Press Center ▶️ begin line tracing at normal speed.  

2. **School Zone (Yellow Line)**  
   ▸ Detect yellow ▶️ ⏸️ stop + 🔴🔴 flash LEDs  
   • 7 s at normal speed  
   • 3 s max-speed boost  

3. **2nd Red Line: Boost**  
   ▸ Instant max-speed boost (2.6 s) ▶️ resume normal  

4. **3rd Red Line: Traffic Light**  
   ▸ Stop ▶️ reverse 10 cm ▶️ wait for “two” from OpenMV ▶️ go  

5. **4th Red Line: Complex Maneuver**  
   • **Stage 1:** Boost 1.5 s  
   • **Stage 2:** Cruise 12.5 s  
   • **Stage 3:** Final boost 1 s  

6. **5th Red Line: Finish**  
   ▸ All motors stop ▶️ “Mission Complete!” ▶️ victory tune 🎉  

────────────────────────────────────────────────────  
🛠 **Hardware Layout**  
┏━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓  
┃ Port          ┃ Device                        ┃ Role                         ┃  
┣━━━━━━━━━━━━━━━╋━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╋━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┫  
┃ Motor A       ┃ EV3 Large Motor               ┃ Drive (forward/reverse)      ┃  
┃ Motor D       ┃ EV3 Large/Medium Motor        ┃ Steering (wheel angle)       ┃  
┃ Sensor 1      ┃ Mindsensors LSA (8-IR)        ┃ Line position sensing (I²C)  ┃  
┃ Sensor 3      ┃ OpenMV Cam (UART 115200 baud) ┃ Traffic-light detection      ┃  
┃ Sensor 4      ┃ EV3 Color Sensor              ┃ Red / Yellow line detection  ┃  
┗━━━━━━━━━━━━━━━┻━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┻━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛  

────────────────────────────────────────────────────  
🔧 **Custom P-Control Logic**  

1. **Group Sensors**  
   • Left = sensors 0–2 | Ignore 3–4 | Right = 5–7  

2. **Compute Averages**  
   left_dark = average of [0,1,2]  
   right_dark = average of [5,6,7]  
   error = left_dark − right_dark  

3. **Dynamic Gain**  
   • Normal speed: Kp = STEERING_KP_LINETRACE  
   • High speed: Kp = STEERING_KP_LINETRACE × HIGH_SPEED_KP_FACTOR  

4. **Clamp & Steer**  
   correction = clamp(−90 … 90, Kp × error)  
   steering_motor.run_target(..., correction)  

────────────────────────────────────────────────────  
⚙️ **Configuration & Tuning**  
• DRIVE_SPEED_LINETRACE: 200–400  
• DRIVE_SPEED_MAX_LINETRACE: safe max per motor  
• STEERING_KP_LINETRACE: −5.0 … −10.0  
• HIGH_SPEED_KP_FACTOR: 0.1 … 0.3  
• *_BOOST_DURATION_MS: tune per mission step  
• RGB thresholds for RED_/YELLOW_: calibrate under track lights  

────────────────────────────────────────────────────  
📄 **License**  
Distributed under the **MIT License** – free to use, modify & share!  

════════════════════════════════════════════════════  

Ready to supercharge your LEGO EV3? Let’s drive! 🚗💨

