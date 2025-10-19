import { useState } from 'react';
import { Card } from './ui/card';
import { Badge } from './ui/badge';
import { Play, Square, Home, Radar, Sparkles, Package, ArrowUp, ArrowDown, RotateCw, Battery, Thermometer, Wifi, Activity } from 'lucide-react';
import { Slider } from './ui/slider';

interface Task {
  id: number;
  name: string;
  priority: 'High' | 'Medium' | 'Low';
  status: 'Pending' | 'Running' | 'Completed';
  estimatedTime: string;
}

export default function CommandCenter() {
  const [manualMode, setManualMode] = useState(false);
  const [speed, setSpeed] = useState([50]);
  
  const [tasks, setTasks] = useState<Task[]>([
    { id: 1, name: 'Soil Analysis - Zone B', priority: 'High', status: 'Running', estimatedTime: '1h 20m' },
    { id: 2, name: 'Solar Panel Inspection', priority: 'Medium', status: 'Pending', estimatedTime: '45m' },
    { id: 3, name: 'Resource Delivery', priority: 'High', status: 'Pending', estimatedTime: '2h 10m' },
    { id: 4, name: 'Return to Base', priority: 'Low', status: 'Pending', estimatedTime: '30m' },
  ]);
  
  const robotHealth = {
    battery: 78,
    internalTemp: 33,
    motorLoad: 'Normal',
    communication: 'Stable',
    aiMode: 'Auto'
  };
  
  const commandHistory = [
    { time: '09:45', command: 'Start Mission', status: 'Executed', result: 'Success' },
    { time: '09:47', command: 'Scan Area', status: 'Executed', result: 'In Progress' },
    { time: '09:50', command: 'Avoid Obstacle', status: 'Executed', result: 'Success' },
    { time: '09:52', command: 'Adjust Path', status: 'Executed', result: 'Success' },
  ];
  
  const getPriorityColor = (priority: string) => {
    switch(priority) {
      case 'High': return 'bg-red-500/20 text-red-400 border-red-500/50';
      case 'Medium': return 'bg-yellow-500/20 text-yellow-400 border-yellow-500/50';
      case 'Low': return 'bg-green-500/20 text-green-400 border-green-500/50';
      default: return 'bg-gray-500/20 text-gray-400 border-gray-500/50';
    }
  };
  
  const getStatusColor = (result: string) => {
    switch(result) {
      case 'Success': return 'text-green-400';
      case 'In Progress': return 'text-yellow-400';
      case 'Failed': return 'text-red-400';
      default: return 'text-gray-400';
    }
  };

  return (
    <div className="space-y-6">
      {/* Mission Control Toolbar */}
      <Card className="bg-[#1F2833]/50 backdrop-blur-sm border-cyan-500/20 p-6">
        <h3 className="text-lg text-[#E5E5E5] mb-4 flex items-center gap-2">
          <Activity className="w-5 h-5 text-cyan-400" />
          Mission Control
        </h3>
        
        <div className="grid grid-cols-2 md:grid-cols-3 lg:grid-cols-6 gap-3">
          <button className="flex flex-col items-center gap-2 p-4 bg-green-500/10 hover:bg-green-500/20 text-[#E5E5E5] rounded-lg border border-green-500/50 hover:border-green-500/70 transition-all hover:shadow-lg hover:shadow-green-500/20">
            <Play className="w-6 h-6 text-green-400" />
            <span className="text-sm">Start Mission</span>
          </button>
          
          <button className="flex flex-col items-center gap-2 p-4 bg-red-500/10 hover:bg-red-500/20 text-[#E5E5E5] rounded-lg border border-red-500/50 hover:border-red-500/70 transition-all hover:shadow-lg hover:shadow-red-500/20">
            <Square className="w-6 h-6 text-red-400" />
            <span className="text-sm">Stop Mission</span>
          </button>
          
          <button className="flex flex-col items-center gap-2 p-4 bg-cyan-500/10 hover:bg-cyan-500/20 text-[#E5E5E5] rounded-lg border border-cyan-500/50 hover:border-cyan-500/70 transition-all hover:shadow-lg hover:shadow-cyan-500/20">
            <Home className="w-6 h-6 text-cyan-400" />
            <span className="text-sm">Return to Base</span>
          </button>
          
          <button className="flex flex-col items-center gap-2 p-4 bg-cyan-500/10 hover:bg-cyan-500/20 text-[#E5E5E5] rounded-lg border border-cyan-500/50 hover:border-cyan-500/70 transition-all hover:shadow-lg hover:shadow-cyan-500/20">
            <Radar className="w-6 h-6 text-cyan-400" />
            <span className="text-sm">Scan Area</span>
          </button>
          
          <button className="flex flex-col items-center gap-2 p-4 bg-cyan-500/10 hover:bg-cyan-500/20 text-[#E5E5E5] rounded-lg border border-cyan-500/50 hover:border-cyan-500/70 transition-all hover:shadow-lg hover:shadow-cyan-500/20">
            <Sparkles className="w-6 h-6 text-cyan-400" />
            <span className="text-sm">Clean Solar Panel</span>
          </button>
          
          <button className="flex flex-col items-center gap-2 p-4 bg-cyan-500/10 hover:bg-cyan-500/20 text-[#E5E5E5] rounded-lg border border-cyan-500/50 hover:border-cyan-500/70 transition-all hover:shadow-lg hover:shadow-cyan-500/20">
            <Package className="w-6 h-6 text-cyan-400" />
            <span className="text-sm">Deliver Package</span>
          </button>
        </div>
      </Card>
      
      {/* Main Grid */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Task Queue Panel */}
        <Card className="lg:col-span-2 bg-[#1F2833]/50 backdrop-blur-sm border-cyan-500/20 p-6">
          <h3 className="text-lg text-[#E5E5E5] mb-4">Task Queue</h3>
          
          <div className="space-y-3">
            {tasks.map((task) => (
              <div 
                key={task.id}
                className="bg-cyan-500/5 rounded-lg p-4 border border-cyan-500/20 hover:border-cyan-500/40 transition-all cursor-move"
              >
                <div className="flex items-start justify-between mb-2">
                  <div className="flex-1">
                    <h4 className="text-[#E5E5E5]">{task.name}</h4>
                    <p className="text-xs text-cyan-400/70 mt-1">Est. Time: {task.estimatedTime}</p>
                  </div>
                  <div className="flex items-center gap-2">
                    <Badge className={`${getPriorityColor(task.priority)} border`}>
                      {task.priority}
                    </Badge>
                    <Badge className={task.status === 'Running' ? 'bg-green-500/20 text-green-400 border border-green-500/50' : task.status === 'Completed' ? 'bg-blue-500/20 text-blue-400 border border-blue-500/50' : 'bg-gray-500/20 text-gray-400 border border-gray-500/50'}>
                      {task.status}
                    </Badge>
                  </div>
                </div>
                
                <div className="flex items-center gap-2 text-xs text-cyan-400/50">
                  <span>Drag to reorder priority</span>
                </div>
              </div>
            ))}
          </div>
        </Card>
        
        {/* Robot Health Panel */}
        <Card className="bg-[#1F2833]/50 backdrop-blur-sm border-cyan-500/20 p-6">
          <h3 className="text-lg text-[#E5E5E5] mb-4">Robot Health</h3>
          
          <div className="space-y-4">
            <div className="bg-cyan-500/5 rounded-lg p-3 border border-cyan-500/10">
              <div className="flex items-center justify-between mb-2">
                <div className="flex items-center gap-2">
                  <Battery className="w-4 h-4 text-cyan-400" />
                  <span className="text-xs text-[#E5E5E5]">Battery</span>
                </div>
                <span className="text-sm text-cyan-400">{robotHealth.battery}%</span>
              </div>
              <div className="w-full bg-cyan-950 rounded-full h-2">
                <div 
                  className="bg-gradient-to-r from-cyan-400 to-green-400 h-2 rounded-full transition-all"
                  style={{ width: `${robotHealth.battery}%` }}
                />
              </div>
            </div>
            
            <div className="bg-cyan-500/5 rounded-lg p-3 border border-cyan-500/10">
              <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <Thermometer className="w-4 h-4 text-cyan-400" />
                  <span className="text-xs text-[#E5E5E5]">Internal Temp</span>
                </div>
                <span className="text-sm text-cyan-400">{robotHealth.internalTemp}°C</span>
              </div>
            </div>
            
            <div className="bg-cyan-500/5 rounded-lg p-3 border border-cyan-500/10">
              <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <Activity className="w-4 h-4 text-cyan-400" />
                  <span className="text-xs text-[#E5E5E5]">Motor Load</span>
                </div>
                <Badge className="bg-green-500/20 text-green-400 border border-green-500/50">
                  {robotHealth.motorLoad}
                </Badge>
              </div>
            </div>
            
            <div className="bg-cyan-500/5 rounded-lg p-3 border border-cyan-500/10">
              <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <Wifi className="w-4 h-4 text-cyan-400" />
                  <span className="text-xs text-[#E5E5E5]">Communication</span>
                </div>
                <Badge className="bg-green-500/20 text-green-400 border border-green-500/50">
                  {robotHealth.communication}
                </Badge>
              </div>
            </div>
            
            <div className="bg-cyan-500/5 rounded-lg p-3 border border-cyan-500/10">
              <div className="flex items-center justify-between">
                <span className="text-xs text-[#E5E5E5]">AI Mode</span>
                <div className="flex items-center gap-2">
                  <button 
                    onClick={() => setManualMode(!manualMode)}
                    className={`px-3 py-1 rounded text-xs transition-colors ${!manualMode ? 'bg-cyan-500/20 text-cyan-400 border border-cyan-500/50' : 'bg-gray-500/10 text-gray-400'}`}
                  >
                    Auto
                  </button>
                  <button 
                    onClick={() => setManualMode(!manualMode)}
                    className={`px-3 py-1 rounded text-xs transition-colors ${manualMode ? 'bg-cyan-500/20 text-cyan-400 border border-cyan-500/50' : 'bg-gray-500/10 text-gray-400'}`}
                  >
                    Manual
                  </button>
                </div>
              </div>
            </div>
          </div>
        </Card>
      </div>
      
      {/* Manual Override & Command History */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* Manual Override Joystick */}
        {manualMode && (
          <Card className="bg-[#1F2833]/50 backdrop-blur-sm border-cyan-500/20 p-6">
            <div className="flex items-center justify-between mb-4">
              <h3 className="text-lg text-[#E5E5E5]">Manual Override</h3>
              <Badge className="bg-yellow-500/20 text-yellow-400 border border-yellow-500/50 animate-pulse">
                Manual Mode Active
              </Badge>
            </div>
            
            <div className="space-y-6">
              {/* Virtual Joystick */}
              <div className="flex items-center justify-center">
                <div className="relative w-64 h-64 bg-cyan-500/5 rounded-full border-2 border-cyan-500/20">
                  {/* Center circle */}
                  <div className="absolute inset-0 flex items-center justify-center">
                    <div className="w-16 h-16 bg-cyan-500/30 rounded-full border-2 border-cyan-400 flex items-center justify-center cursor-pointer hover:bg-cyan-500/50 transition-colors">
                      <div className="w-8 h-8 bg-cyan-400 rounded-full" />
                    </div>
                  </div>
                  
                  {/* Direction buttons */}
                  <button className="absolute top-4 left-1/2 -translate-x-1/2 w-12 h-12 bg-cyan-500/20 hover:bg-cyan-500/40 rounded-lg border border-cyan-500/50 flex items-center justify-center transition-colors">
                    <ArrowUp className="w-6 h-6 text-cyan-400" />
                  </button>
                  <button className="absolute bottom-4 left-1/2 -translate-x-1/2 w-12 h-12 bg-cyan-500/20 hover:bg-cyan-500/40 rounded-lg border border-cyan-500/50 flex items-center justify-center transition-colors">
                    <ArrowDown className="w-6 h-6 text-cyan-400" />
                  </button>
                  <button className="absolute top-1/2 -translate-y-1/2 left-4 w-12 h-12 bg-cyan-500/20 hover:bg-cyan-500/40 rounded-lg border border-cyan-500/50 flex items-center justify-center transition-colors">
                    <ArrowUp className="w-6 h-6 text-cyan-400 -rotate-90" />
                  </button>
                  <button className="absolute top-1/2 -translate-y-1/2 right-4 w-12 h-12 bg-cyan-500/20 hover:bg-cyan-500/40 rounded-lg border border-cyan-500/50 flex items-center justify-center transition-colors">
                    <ArrowUp className="w-6 h-6 text-cyan-400 rotate-90" />
                  </button>
                  
                  {/* Rotate button */}
                  <button className="absolute top-1/2 left-1/2 -translate-x-1/2 translate-y-16 w-12 h-12 bg-cyan-500/20 hover:bg-cyan-500/40 rounded-lg border border-cyan-500/50 flex items-center justify-center transition-colors">
                    <RotateCw className="w-6 h-6 text-cyan-400" />
                  </button>
                </div>
              </div>
              
              {/* Speed Control */}
              <div className="space-y-2">
                <div className="flex justify-between text-sm">
                  <span className="text-[#E5E5E5]">Speed Control</span>
                  <span className="text-cyan-400">{speed[0]}%</span>
                </div>
                <Slider 
                  value={speed} 
                  onValueChange={setSpeed}
                  max={100}
                  step={1}
                  className="w-full"
                />
              </div>
            </div>
          </Card>
        )}
        
        {/* Command History Log */}
        <Card className={`bg-[#1F2833]/50 backdrop-blur-sm border-cyan-500/20 p-6 ${manualMode ? '' : 'lg:col-span-2'}`}>
          <h3 className="text-lg text-[#E5E5E5] mb-4">Command History</h3>
          
          <div className="overflow-x-auto">
            <table className="w-full">
              <thead>
                <tr className="border-b border-cyan-500/20">
                  <th className="text-left text-xs text-cyan-400/70 pb-2">Time</th>
                  <th className="text-left text-xs text-cyan-400/70 pb-2">Command</th>
                  <th className="text-left text-xs text-cyan-400/70 pb-2">Status</th>
                  <th className="text-left text-xs text-cyan-400/70 pb-2">Result</th>
                </tr>
              </thead>
              <tbody>
                {commandHistory.map((entry, index) => (
                  <tr key={index} className="border-b border-cyan-500/10">
                    <td className="py-3 text-sm text-[#E5E5E5]">{entry.time}</td>
                    <td className="py-3 text-sm text-[#E5E5E5]">{entry.command}</td>
                    <td className="py-3">
                      <Badge className="bg-cyan-500/20 text-cyan-400 border border-cyan-500/50">
                        {entry.status}
                      </Badge>
                    </td>
                    <td className={`py-3 text-sm ${getStatusColor(entry.result)}`}>
                      {entry.result}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </Card>
      </div>
    </div>
  );
}
