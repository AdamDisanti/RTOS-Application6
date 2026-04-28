Application demo on a github pages website:
https://adamdisanti.github.io/RTOS-Application6/

Youtube link for the demo:
https://www.youtube.com/watch?v=EOkDo1x-bsY

Application 6
4-27-26
Adam Disanti
5474081

Overview
-This project continues my aerospace/defense theme by building a real-time hazard response system in Wokwi. The system uses a potentiometer as a simulated hazard intensity input, a pushbutton as a pilot override input, a green LED as the system heartbeat, and a red LED as the threat indicator. The design uses an ISR, multiple FreeRTOS tasks, a binary semaphore, a counting semaphore, a queue, and a mutex. The goal is to prove that the hard override path meets its deadline while the rest of the system still behaves predictably under load.

Company Synopsis
-I framed this application as an Orlando aerospace/defense prototype similar to the type of pilot-assist or training-system software a company like Lockheed Martin might develop. In this scenario, timing matters as much as correctness because the system must react immediately to a pilot override command while still monitoring hazard intensity in real time. A missed hard deadline on the override path would be unacceptable because it could delay a critical response. This system focuses on determinism, priority separation, and measurable timing behavior.

Task Table
Heartbeat Task  
- Period: 500ms on / 500ms off  
- Classification: Soft  
- Consequence if missed: unknown if system is still alive and firing properly

Sensor Acquisition Task  
- Period: 20ms  
- Classification: Soft  
- Consequence if missed: stale hazard samples and slow hazard detection  

Threat Assessment Task  
- Period: event-driven from queue  
- Classification: Soft  
- Consequence if missed: slower hazard severity analysis  
- Notes: this is the variable-execution-time task  

Threat Alarm Task  
- Period: event-driven from counting semaphore  
- Classification: Soft  
- Consequence if missed: delayed alarm indication, but not immediate system failure  

Hard Override Response Task  
- Period: event-driven from ISR / binary semaphore  
- Classification: Hard  
- Deadline: 5000us  
- Consequence if missed: delayed pilot override response 

Telemetry Task  
- Period: 250ms  
- Classification: Soft  
- Consequence if missed: delayed UART visibility into system behavior  

Pilot Override ISR  
- Triggered by the pushbutton  
- Role: signal the hard response task immediately  
- Consequence if delayed: delayed start of the hard override path  

Engineering Analysis

#1. Scheduler Fit
-I assigned the Hard Override Response Task the highest priority in the system and restricted it to critical deadline work. The hard task no longer blinked the red LED or handled threat alarm behavior like application 4. Instead, it responded to the ISR, pulsed a response strobe, toggled defense mode, and recorded timing. This separation was needed to make predictable execution on the hard path.
-The remaining tasks were intentionally left as soft tasks. The heartbeat, telemetry, threat assessment, and threat alarm tasks are allowed to execute later or be delayed without causing system failure. The threat assessment task has variable execution time and runs through a queue which does not block the hard override response.
-Firmware timing logs showed override latencies of 15us, 33us, and 50us with a declared hard deadline of 5000us. The worst reported override latency during stable testing was 50us, leaving roughly 4950us of margin before a deadline miss.
-I also verified this behavior using the Wokwi logic analyzer and GTKWave. In the waveform capture, the pilot override button transition (D0) is immediately followed by the ISR strobe (D3) and then the hard response strobe (D4). The measured time between the ISR strobe and the response strobe was on the order of tens of microseconds, matching the firmware-reported latencies and remaining well below the 5000us hard deadline. This shows hardware level confirmation that the scheduler and task priorities correctly protect the hard path.

#2. Race-Proofing
-The main race risk in this design is shared state and shared UART output. Multiple tasks read or update values such as the latest hazard reading, severity, defense mode, alert counts, override counts, queue statistics, and the last-event string. Without protection, telemetry output could become inconsistent or misleading.
-I protected shared state and output with the mutex `xStateMutex`. All shared-state updates and snapshots occur inside mutex-protected sections, including helper functions such as `set_latest_hazard()`, `increment_alert_count()`, `increment_override_count()`, and `note_override_latency()`, as well as the telemetry snapshot before printing.
-The mutex ensures that concurrent access from the telemetry task and the event-driven tasks does not corrupt shared state or interleave UART output.

#3. Worst-Case Spike
-The heaviest load applied to the prototype involved repeated hazard threshold crossings while the variable-execution-time threat assessment task was active. An earlier version of the design allowed the hard override task to perform blocking work, which caused unstable behavior and missed deadlines.
-In the final version, the hard override task was isolated from threat blinking and variable computation. Under load, the internal queue remained healthy, with `queue_high_water=1` and `queue_drops=0` throughout testing.
-The worst override latency observed during active system behavior was 50us with a declared hard deadline of 5000us. This leaves approximately 4950us of margin, showing that even during a worst-case spike, the hard path remains safe.
-The GTKWave capture reinforces this result visually. While the variable-load task (D5) shows dense and irregular activity, the hard response strobe (D4) remains short and unaffected, demonstrating strong isolation between soft and hard execution paths.

#4. Design Trade-off
-One feature I purposely did not keep from Application 5 was the WiFi/webserver interface. Application 6 only required one outward communication link, and UART was enough for the this system. Removing the web stack simplified timing analysis and reduced noncritical interference.
-This was perfect for the context of aerospace/defense because guaranteeing the pilot override deadline is more important than providing a rich remote UI. Simplifying the design made the hard real-time behavior easier to reason about and verify.
-I also separated the threat alarm behavior into its own soft task instead of letting the hard override task handle it. This reduced coupling between features and ensured that noncritical LED blinking could never delay a deadline-critical response.

#5. Domain Reflection
-Using a hard deadline on the pilot override path reflects the aerospace/defense system requirements. In a pilot-assist or training-system context, an override command must be handled immediately, regardless of background processing.
-The defense (shielded) mode reinforces this context even more. When the pilot override is activated, the system suppresses threat alarms instead of continuing to blink the red LED. In final testing, I confirmed this behavior with:
`[THREAT SUPPRESSED] ... reason=shielded mode enabled`
-This shows that the override input does not toggle a variable. The override actually changes system behavior.
-If extended into a more industrial version, the next step would be adding a latched event-history and acknowledgment mechanism so that important hazard and override events could be reviewed and explicitly acknowledged.

AI citation
-I used this chat session as an AI design assistant during development. AI helped identify architectural issues in the initial Application 6 design and guided the restructuring which helped isolate the hard override path, improved debounce behavior, and stabilized variable-load processing. AI also assisted with the brainstorming for this project to keep in a similar context to my other space system themed applications. All timing claims and validation results in this README are based on my own Wokwi serial output and GTKWave logic-analyzer captures. I had trouble getting pulsewave to work on windows and mac so I ended up using GTKwave which AI recommended after about 30 minutes of troubleshooting pulseview.
