#!/usr/bin/env python3
"""
Simulation Monitor and Auto-Restart Script
Monitors the execution of realistic_livox_simulator.py and restarts if needed
"""

import subprocess
import time
import os
import sys
import psutil
import threading
from datetime import datetime
import signal

class SimulationMonitor:
    def __init__(self, script_path, timeout_seconds=60, max_attempts=10):
        self.script_path = script_path
        self.timeout_seconds = timeout_seconds
        self.max_attempts = max_attempts
        self.current_process = None
        self.attempt_count = 0
        self.success = False
        self.start_time = None
        
    def log_message(self, message):
        """Log message with timestamp"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{timestamp}] {message}")
        
        # Also log to file
        log_file = "logs/simulation_monitor.log"
        with open(log_file, "a", encoding="utf-8") as f:
            f.write(f"[{timestamp}] {message}\n")
    
    def kill_process_tree(self, pid):
        """Kill process and all its children"""
        try:
            parent = psutil.Process(pid)
            children = parent.children(recursive=True)
            
            # Kill children first
            for child in children:
                try:
                    child.terminate()
                except psutil.NoSuchProcess:
                    pass
            
            # Wait for children to terminate
            gone, alive = psutil.wait_procs(children, timeout=3)
            
            # Force kill any remaining children
            for p in alive:
                try:
                    p.kill()
                except psutil.NoSuchProcess:
                    pass
            
            # Kill parent
            try:
                parent.terminate()
                parent.wait(timeout=3)
            except (psutil.NoSuchProcess, psutil.TimeoutExpired):
                try:
                    parent.kill()
                except psutil.NoSuchProcess:
                    pass
                    
        except psutil.NoSuchProcess:
            pass
    
    def monitor_process(self, process):
        """Monitor process execution with timeout"""
        start_time = time.time()
        
        while True:
            # Check if process is still running
            if process.poll() is not None:
                # Process finished
                return process.returncode
            
            # Check timeout
            elapsed = time.time() - start_time
            if elapsed > self.timeout_seconds:
                self.log_message(f"⏰ TIMEOUT: Process exceeded {self.timeout_seconds} seconds")
                
                # Kill the process tree
                try:
                    self.kill_process_tree(process.pid)
                    self.log_message("🔪 Process tree terminated due to timeout")
                except Exception as e:
                    self.log_message(f"⚠️ Error killing process: {e}")
                
                return -1  # Timeout return code
            
            # Log progress every 10 seconds
            if int(elapsed) % 10 == 0 and int(elapsed) > 0:
                try:
                    # Get memory usage
                    proc_info = psutil.Process(process.pid)
                    memory_mb = proc_info.memory_info().rss / 1024 / 1024
                    cpu_percent = proc_info.cpu_percent()
                    
                    self.log_message(f"📊 Progress: {elapsed:.0f}s elapsed, Memory: {memory_mb:.1f}MB, CPU: {cpu_percent:.1f}%")
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
            
            time.sleep(1)
    
    def run_single_attempt(self):
        """Run a single simulation attempt"""
        self.attempt_count += 1
        
        self.log_message(f"🚀 Starting attempt {self.attempt_count}/{self.max_attempts}")
        self.log_message(f"📝 Script: {self.script_path}")
        self.log_message(f"⏱️ Timeout: {self.timeout_seconds} seconds")
        
        try:
            # Start the process
            cmd = [sys.executable, self.script_path]
            
            self.log_message(f"🔧 Command: {' '.join(cmd)}")
            
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                cwd=os.path.dirname(self.script_path)
            )
            
            self.current_process = process
            
            # Monitor output in a separate thread
            output_lines = []
            
            def read_output():
                try:
                    for line in iter(process.stdout.readline, ''):
                        if line:
                            line = line.strip()
                            output_lines.append(line)
                            self.log_message(f"📤 {line}")
                except Exception as e:
                    self.log_message(f"⚠️ Error reading output: {e}")
            
            output_thread = threading.Thread(target=read_output)
            output_thread.daemon = True
            output_thread.start()
            
            # Monitor the process
            return_code = self.monitor_process(process)
            
            # Wait for output thread to finish
            output_thread.join(timeout=2)
            
            # Analyze results
            if return_code == 0:
                self.log_message(f"✅ SUCCESS: Simulation completed successfully!")
                
                # Check for output files
                output_indicators = [
                    "realistic_lidar_output",
                    "fast_realistic_lidar_output",
                    "simple_lidar_output",
                    "Simulation completed",
                    "Results saved"
                ]
                
                output_text = ' '.join(output_lines)
                has_output = any(indicator in output_text for indicator in output_indicators)
                
                if has_output:
                    self.log_message(f"📁 Output files detected")
                    self.success = True
                    return True
                else:
                    self.log_message(f"⚠️ No output files detected, treating as failure")
                    return False
                    
            elif return_code == -1:
                self.log_message(f"⏰ TIMEOUT: Process exceeded time limit")
                return False
            else:
                self.log_message(f"❌ FAILURE: Process exited with code {return_code}")
                return False
                
        except KeyboardInterrupt:
            self.log_message(f"⚠️ INTERRUPTED: User cancelled execution")
            if self.current_process:
                try:
                    self.kill_process_tree(self.current_process.pid)
                except:
                    pass
            return False
            
        except Exception as e:
            self.log_message(f"❌ ERROR: {str(e)}")
            return False
        
        finally:
            self.current_process = None
    
    def run_with_restart(self):
        """Run simulation with automatic restart on failure"""
        self.start_time = time.time()
        
        self.log_message("="*80)
        self.log_message("🎯 SIMULATION MONITOR STARTED")
        self.log_message(f"📝 Script: {os.path.basename(self.script_path)}")
        self.log_message(f"⏱️ Timeout per attempt: {self.timeout_seconds} seconds")
        self.log_message(f"🔄 Maximum attempts: {self.max_attempts}")
        self.log_message("="*80)
        
        while self.attempt_count < self.max_attempts and not self.success:
            attempt_start = time.time()
            
            success = self.run_single_attempt()
            
            attempt_duration = time.time() - attempt_start
            
            if success:
                total_duration = time.time() - self.start_time
                self.log_message("="*80)
                self.log_message(f"🎉 FINAL SUCCESS!")
                self.log_message(f"✅ Completed in attempt {self.attempt_count}/{self.max_attempts}")
                self.log_message(f"⏱️ Attempt duration: {attempt_duration:.1f} seconds")
                self.log_message(f"⏱️ Total duration: {total_duration:.1f} seconds")
                self.log_message("="*80)
                break
            else:
                self.log_message(f"❌ Attempt {self.attempt_count} failed after {attempt_duration:.1f} seconds")
                
                if self.attempt_count < self.max_attempts:
                    wait_time = min(5, self.attempt_count)  # Progressive wait
                    self.log_message(f"⏳ Waiting {wait_time} seconds before retry...")
                    time.sleep(wait_time)
                    
                    # Reduce timeout for subsequent attempts
                    if self.attempt_count >= 3:
                        self.timeout_seconds = max(30, self.timeout_seconds - 10)
                        self.log_message(f"⚡ Reducing timeout to {self.timeout_seconds} seconds")
        
        # Final summary
        total_duration = time.time() - self.start_time
        
        if self.success:
            self.log_message(f"\n🏆 MISSION ACCOMPLISHED!")
            self.log_message(f"📊 Final Statistics:")
            self.log_message(f"   • Successful attempt: {self.attempt_count}/{self.max_attempts}")
            self.log_message(f"   • Total time: {total_duration:.1f} seconds")
            self.log_message(f"   • Average time per attempt: {total_duration/self.attempt_count:.1f} seconds")
            
            # Look for output directories
            possible_outputs = [
                "realistic_lidar_output",
                "fast_realistic_lidar_output",
                "simple_lidar_output"
            ]
            
            for output_dir in possible_outputs:
                if os.path.exists(output_dir):
                    self.log_message(f"📁 Output found: {output_dir}")
                    
                    # List key files
                    try:
                        files = os.listdir(output_dir)
                        key_files = [f for f in files if f.endswith(('.pcd', '.lvx', '.csv', '.las'))]
                        if key_files:
                            self.log_message(f"   📄 Key files: {', '.join(key_files[:5])}")
                            if len(key_files) > 5:
                                self.log_message(f"   📄 ... and {len(key_files)-5} more files")
                    except:
                        pass
            
            return True
        else:
            self.log_message(f"\n💥 MISSION FAILED!")
            self.log_message(f"❌ All {self.max_attempts} attempts failed")
            self.log_message(f"⏱️ Total time spent: {total_duration:.1f} seconds")
            self.log_message(f"\n💡 Suggestions:")
            self.log_message(f"   • Check system resources (RAM, CPU)")
            self.log_message(f"   • Reduce simulation parameters")
            self.log_message(f"   • Check for missing dependencies")
            self.log_message(f"   • Review error logs above")
            
            return False

def main():
    """Main monitoring function"""
    print("Livox Simulator Monitor v1.0")
    print("Automatic execution monitoring with restart capability")
    print("=" * 60)
    
    # Determine which script to run
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Prioritize optimized versions for better performance
    possible_scripts = [
        os.path.join(script_dir, "fast_realistic_livox_simulator.py"),
        os.path.join(script_dir, "simple_optimized_simulator.py"),
        os.path.join(script_dir, "realistic_livox_simulator.py")
    ]
    
    script_to_run = None
    for script in possible_scripts:
        if os.path.exists(script):
            script_to_run = script
            break
    
    if not script_to_run:
        print("❌ ERROR: No simulation script found!")
        print("Expected files:")
        for script in possible_scripts:
            print(f"  • {script}")
        return False
    
    print(f"📝 Selected script: {os.path.basename(script_to_run)}")
    
    # Parse command line arguments
    timeout_seconds = 60  # Default timeout
    max_attempts = 8      # Default max attempts
    
    i = 1
    while i < len(sys.argv):
        if sys.argv[i] == '--timeout' and i + 1 < len(sys.argv):
            try:
                timeout_seconds = int(sys.argv[i + 1])
                if timeout_seconds <= 0:
                    print("⚠️ Invalid timeout value, using default: 60 seconds")
                    timeout_seconds = 60
                else:
                    print(f"⏱️ Custom timeout: {timeout_seconds} seconds")
                i += 2
            except ValueError:
                print("⚠️ Invalid timeout value, using default: 60 seconds")
                i += 2
        elif sys.argv[i] == '--max-attempts' and i + 1 < len(sys.argv):
            try:
                max_attempts = int(sys.argv[i + 1])
                if max_attempts <= 0:
                    print("⚠️ Invalid max attempts value, using default: 8")
                    max_attempts = 8
                else:
                    print(f"🔄 Custom max attempts: {max_attempts}")
                i += 2
            except ValueError:
                print("⚠️ Invalid max attempts value, using default: 8")
                i += 2
        else:
            i += 1
    
    # Create and run monitor
    monitor = SimulationMonitor(
        script_path=script_to_run,
        timeout_seconds=timeout_seconds,
        max_attempts=max_attempts
    )
    
    try:
        success = monitor.run_with_restart()
        
        if success:
            print("\n🎉 Monitoring completed successfully!")
            print("Check the log file 'simulation_monitor.log' for detailed information.")
            return True
        else:
            print("\n💥 Monitoring failed after all attempts.")
            print("Check the log file 'simulation_monitor.log' for detailed error information.")
            return False
            
    except KeyboardInterrupt:
        print("\n⚠️ Monitoring interrupted by user")
        if monitor.current_process:
            try:
                monitor.kill_process_tree(monitor.current_process.pid)
                print("🔪 Terminated running simulation")
            except:
                pass
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)