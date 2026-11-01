#!/usr/bin/env python3
"""Sample a process tree and simulation clock into one compact JSON record."""
import argparse,json,signal,time
from pathlib import Path
import psutil,rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from benchmark_geometry import cpu_delta

p=argparse.ArgumentParser();p.add_argument('--pids',nargs='+',type=int,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();running=True
rclpy.init();signal.signal(signal.SIGUSR1,lambda *_:globals().__setitem__('running',False));node=Node('v0_resource_monitor');clocks=[];node.create_subscription(Clock,'/clock',lambda m:clocks.append(m.clock.sec+m.clock.nanosec*1e-9),10);start=time.monotonic();rss=[];previous_cpu={};cpu_seconds=0.
while running:
 processes=[]
 for pid in a.pids:
  try:
   root=psutil.Process(pid);processes.append(root);processes.extend(root.children(recursive=True))
  except (psutil.NoSuchProcess,psutil.AccessDenied):pass
 unique={process.pid:process for process in processes}.values();total_rss=0;samples={}
 for process in unique:
  try:
   total_rss+=process.memory_info().rss;usage=process.cpu_times();samples[(process.pid,process.create_time())]=usage.user+usage.system
  except (psutil.NoSuchProcess,psutil.AccessDenied):pass
 cpu_seconds+=cpu_delta(previous_cpu,samples)
 if unique:rss.append(total_rss)
 rclpy.spin_once(node,timeout_sec=.5)
 time.sleep(.4)
wall=time.monotonic()-start;summary={'samples':len(rss),'wall_duration_s':wall,'max_rss_mb':max(rss)/1048576 if rss else None,'mean_cpu_percent':cpu_seconds/wall*100 if len(rss)>1 else None,'sim_duration_s':clocks[-1]-clocks[0] if len(clocks)>1 else None}
summary['real_time_factor']=summary['sim_duration_s']/wall if summary['sim_duration_s'] is not None and wall>0 else None;a.output.write_text(json.dumps(summary,indent=2)+'\n');node.destroy_node();rclpy.shutdown()
