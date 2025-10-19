import { Battery, Wifi, MapPin, Thermometer, Activity, Map, Gamepad2, Bell, Settings, AlertTriangle, Sun, Wind, Radiation } from 'lucide-react';
import { Card } from './ui/card';
import { Progress } from './ui/progress';
import { Badge } from './ui/badge';

// Mock data for LunaBots
const lunaBots = [
  {
    id: 1,
    name: 'LunaBot-01',
    mission: 'Soil Scan - Habitat Zone B',
    status: 'active',
    battery: 78,
    connectivity: 'strong',
    location: '23.4°N, 12.8°E',
    temperature: 32,
    lastSync: '2m ago'
  },
  {
    id: 2,
    name: 'LunaBot-02',
    mission: 'Resource Delivery - Mining Site',
    status: 'active',
    battery: 92,
    connectivity: 'strong',
    location: '24.1°N, 13.2°E',
    temperature: 28,
    lastSync: '1m ago'
  },
  {
    id: 3,
    name: 'LunaBot-03',
    mission: 'Solar Panel Maintenance',
    status: 'idle',
    battery: 45,
    connectivity: 'weak',
    location: '23.8°N, 12.5°E',
    temperature: 35,
    lastSync: '5m ago'
  },
  {
    id: 4,
    name: 'LunaBot-04',
    mission: 'Exploration - Crater Alpha',
    status: 'fault',
    battery: 15,
    connectivity: 'lost',
    location: '22.9°N, 11.8°E',
    temperature: 40,
    lastSync: '15m ago'
  }
];

const currentMission = {
  name: 'Soil Analysis - Habitat Zone B',
  progress: 65,
  timeElapsed: '2h 15m',
  timeRemaining: '1h 20m',
  eta: '14:35 UTC',
  type: 'Exploration'
};

const environment = {
  externalTemp: -23,
  radiation: 'Low',
  dustStorm: false,
  solarExposure: 'Day Cycle',
  visibility: 'Clear'
};

export default function MissionDashboard() {
  const getStatusColor = (status: string) => {
    switch(status) {
      case 'active': return 'bg-green-500/20 text-green-400 border-green-500/50';
      case 'idle': return 'bg-yellow-500/20 text-yellow-400 border-yellow-500/50';
      case 'fault': return 'bg-red-500/20 text-red-400 border-red-500/50';
      default: return 'bg-gray-500/20 text-gray-400 border-gray-500/50';
    }
  };
  
  const getStatusIcon = (status: string) => {
    switch(status) {
      case 'active': return '✅';
      case 'idle': return '⚠️';
      case 'fault': return '🔴';
      default: return '⚪';
    }
  };
  
  const getConnectivityColor = (connectivity: string) => {
    switch(connectivity) {
      case 'strong': return 'text-green-400';
      case 'weak': return 'text-yellow-400';
      case 'lost': return 'text-red-400';
      default: return 'text-gray-400';
    }
  };

  return (
    <div className="space-y-6">
      {/* LunaBot Overview Cards */}
      <div>
        <h2 className="text-xl text-[#E5E5E5] mb-4 flex items-center gap-2">
          <Activity className="w-5 h-5 text-cyan-400" />
          LunaBot Fleet Overview
        </h2>
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
          {lunaBots.map((bot) => (
            <Card 
              key={bot.id}
              className="bg-[#1F2833]/50 backdrop-blur-sm border-cyan-500/20 p-5 hover:border-cyan-500/40 transition-all hover:shadow-lg hover:shadow-cyan-500/10"
            >
              <div className="space-y-3">
                {/* Header */}
                <div className="flex items-start justify-between">
                  <div>
                    <h3 className="text-[#E5E5E5]">{bot.name}</h3>
                    <p className="text-xs text-cyan-400/70 mt-1">{bot.mission}</p>
                  </div>
                  <Badge className={`${getStatusColor(bot.status)} border`}>
                    {getStatusIcon(bot.status)} {bot.status}
                  </Badge>
                </div>
                
                {/* Stats Grid */}
                <div className="space-y-2 pt-2 border-t border-cyan-500/10">
                  <div className="flex items-center justify-between">
                    <div className="flex items-center gap-2">
                      <Battery className="w-4 h-4 text-cyan-400" />
                      <span className="text-xs text-[#E5E5E5]">Battery</span>
                    </div>
                    <span className="text-xs text-cyan-400">{bot.battery}%</span>
                  </div>
                  <Progress value={bot.battery} className="h-1.5 bg-cyan-950" />
                  
                  <div className="flex items-center justify-between">
                    <div className="flex items-center gap-2">
                      <Wifi className={`w-4 h-4 ${getConnectivityColor(bot.connectivity)}`} />
                      <span className="text-xs text-[#E5E5E5]">Signal</span>
                    </div>
                    <span className={`text-xs ${getConnectivityColor(bot.connectivity)} capitalize`}>
                      {bot.connectivity}
                    </span>
                  </div>
                  
                  <div className="flex items-center justify-between">
                    <div className="flex items-center gap-2">
                      <MapPin className="w-4 h-4 text-cyan-400" />
                      <span className="text-xs text-[#E5E5E5]">Location</span>
                    </div>
                    <span className="text-xs text-cyan-400">{bot.location}</span>
                  </div>
                  
                  <div className="flex items-center justify-between">
                    <div className="flex items-center gap-2">
                      <Thermometer className="w-4 h-4 text-cyan-400" />
                      <span className="text-xs text-[#E5E5E5]">Temp</span>
                    </div>
                    <span className="text-xs text-cyan-400">{bot.temperature}°C</span>
                  </div>
                  
                  <div className="text-xs text-cyan-400/50 pt-1">
                    Last sync: {bot.lastSync}
                  </div>
                </div>
              </div>
            </Card>
          ))}
        </div>
      </div>
      
      {/* Mission Status & Environment */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Mission Status Widget */}
        <Card className="lg:col-span-2 bg-[#1F2833]/50 backdrop-blur-sm border-cyan-500/20 p-6">
          <div className="flex items-center gap-2 mb-4">
            <Activity className="w-5 h-5 text-cyan-400" />
            <h3 className="text-lg text-[#E5E5E5]">Active Mission Status</h3>
          </div>
          
          <div className="space-y-4">
            <div>
              <div className="flex items-center justify-between mb-2">
                <span className="text-[#E5E5E5]">{currentMission.name}</span>
                <Badge className="bg-cyan-500/20 text-cyan-400 border border-cyan-500/50">
                  {currentMission.type}
                </Badge>
              </div>
              <div className="relative">
                <Progress value={currentMission.progress} className="h-3 bg-cyan-950" />
                <div className="absolute inset-0 flex items-center justify-center">
                  <span className="text-xs text-cyan-400">{currentMission.progress}%</span>
                </div>
              </div>
            </div>
            
            <div className="grid grid-cols-3 gap-4 pt-2">
              <div className="bg-cyan-500/5 rounded-lg p-3 border border-cyan-500/10">
                <p className="text-xs text-cyan-400/70">Time Elapsed</p>
                <p className="text-lg text-cyan-400 mt-1">{currentMission.timeElapsed}</p>
              </div>
              <div className="bg-cyan-500/5 rounded-lg p-3 border border-cyan-500/10">
                <p className="text-xs text-cyan-400/70">Remaining</p>
                <p className="text-lg text-cyan-400 mt-1">{currentMission.timeRemaining}</p>
              </div>
              <div className="bg-cyan-500/5 rounded-lg p-3 border border-cyan-500/10">
                <p className="text-xs text-cyan-400/70">ETA</p>
                <p className="text-lg text-cyan-400 mt-1">{currentMission.eta}</p>
              </div>
            </div>
            
            <div className="flex items-center gap-2 text-xs text-cyan-400/70 bg-cyan-500/5 rounded-lg p-3 border border-cyan-500/10">
              <Activity className="w-4 h-4" />
              <span>AI Prediction: Mission will complete 5 minutes ahead of schedule</span>
            </div>
          </div>
        </Card>
        
        {/* Environmental Readings */}
        <Card className="bg-[#1F2833]/50 backdrop-blur-sm border-cyan-500/20 p-6">
          <div className="flex items-center gap-2 mb-4">
            <Wind className="w-5 h-5 text-cyan-400" />
            <h3 className="text-lg text-[#E5E5E5]">Environment</h3>
          </div>
          
          <div className="space-y-3">
            <div className="flex items-center justify-between p-3 bg-cyan-500/5 rounded-lg border border-cyan-500/10">
              <div className="flex items-center gap-2">
                <Thermometer className="w-4 h-4 text-cyan-400" />
                <span className="text-xs text-[#E5E5E5]">External Temp</span>
              </div>
              <span className="text-sm text-cyan-400">{environment.externalTemp}°C</span>
            </div>
            
            <div className="flex items-center justify-between p-3 bg-cyan-500/5 rounded-lg border border-cyan-500/10">
              <div className="flex items-center gap-2">
                <Radiation className="w-4 h-4 text-cyan-400" />
                <span className="text-xs text-[#E5E5E5]">Radiation</span>
              </div>
              <Badge className="bg-green-500/20 text-green-400 border border-green-500/50">
                {environment.radiation}
              </Badge>
            </div>
            
            <div className="flex items-center justify-between p-3 bg-cyan-500/5 rounded-lg border border-cyan-500/10">
              <div className="flex items-center gap-2">
                <AlertTriangle className="w-4 h-4 text-cyan-400" />
                <span className="text-xs text-[#E5E5E5]">Dust Storm</span>
              </div>
              <Badge className="bg-green-500/20 text-green-400 border border-green-500/50">
                {environment.dustStorm ? 'Active' : 'Clear'}
              </Badge>
            </div>
            
            <div className="flex items-center justify-between p-3 bg-cyan-500/5 rounded-lg border border-cyan-500/10">
              <div className="flex items-center gap-2">
                <Sun className="w-4 h-4 text-cyan-400" />
                <span className="text-xs text-[#E5E5E5]">Solar Exposure</span>
              </div>
              <span className="text-sm text-cyan-400">{environment.solarExposure}</span>
            </div>
          </div>
        </Card>
      </div>
      
      {/* Mini Map & Quick Access */}
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        {/* Mini Map Preview */}
        <Card className="lg:col-span-2 bg-[#1F2833]/50 backdrop-blur-sm border-cyan-500/20 p-6">
          <div className="flex items-center justify-between mb-4">
            <div className="flex items-center gap-2">
              <Map className="w-5 h-5 text-cyan-400" />
              <h3 className="text-lg text-[#E5E5E5]">Fleet Positions</h3>
            </div>
            <button className="px-4 py-2 bg-cyan-500/20 hover:bg-cyan-500/30 text-cyan-400 rounded-lg border border-cyan-500/50 transition-colors text-sm">
              View Full Map
            </button>
          </div>
          
          <div className="relative bg-[#0B0C10] rounded-lg border border-cyan-500/20 h-64 overflow-hidden">
            {/* Simulated map with grid */}
            <div className="absolute inset-0" style={{
              backgroundImage: 'linear-gradient(rgba(0, 209, 255, 0.1) 1px, transparent 1px), linear-gradient(90deg, rgba(0, 209, 255, 0.1) 1px, transparent 1px)',
              backgroundSize: '40px 40px'
            }}>
              {/* Bot positions */}
              {lunaBots.map((bot, index) => (
                <div 
                  key={bot.id}
                  className="absolute w-3 h-3 bg-cyan-400 rounded-full shadow-lg shadow-cyan-500/50 animate-pulse"
                  style={{
                    left: `${20 + index * 20}%`,
                    top: `${30 + index * 10}%`
                  }}
                >
                  <div className="absolute -top-6 left-1/2 -translate-x-1/2 whitespace-nowrap text-xs text-cyan-400">
                    {bot.name}
                  </div>
                </div>
              ))}
              
              {/* Base marker */}
              <div className="absolute w-6 h-6 border-2 border-green-400 rounded-full left-1/2 top-1/2 -translate-x-1/2 -translate-y-1/2">
                <div className="absolute -top-6 left-1/2 -translate-x-1/2 whitespace-nowrap text-xs text-green-400">
                  Base
                </div>
              </div>
            </div>
          </div>
        </Card>
        
        {/* Quick Access Panel */}
        <Card className="bg-[#1F2833]/50 backdrop-blur-sm border-cyan-500/20 p-6">
          <h3 className="text-lg text-[#E5E5E5] mb-4">Quick Access</h3>
          
          <div className="space-y-3">
            <button className="w-full flex items-center gap-3 p-3 bg-cyan-500/10 hover:bg-cyan-500/20 text-[#E5E5E5] rounded-lg border border-cyan-500/20 hover:border-cyan-500/40 transition-all">
              <Map className="w-5 h-5 text-cyan-400" />
              <span className="text-sm">Open Navigation Panel</span>
            </button>
            
            <button className="w-full flex items-center gap-3 p-3 bg-cyan-500/10 hover:bg-cyan-500/20 text-[#E5E5E5] rounded-lg border border-cyan-500/20 hover:border-cyan-500/40 transition-all">
              <Gamepad2 className="w-5 h-5 text-cyan-400" />
              <span className="text-sm">Command Center</span>
            </button>
            
            <button className="w-full flex items-center gap-3 p-3 bg-cyan-500/10 hover:bg-cyan-500/20 text-[#E5E5E5] rounded-lg border border-cyan-500/20 hover:border-cyan-500/40 transition-all">
              <Bell className="w-5 h-5 text-cyan-400" />
              <span className="text-sm">Alerts & Reports</span>
            </button>
            
            <button className="w-full flex items-center gap-3 p-3 bg-cyan-500/10 hover:bg-cyan-500/20 text-[#E5E5E5] rounded-lg border border-cyan-500/20 hover:border-cyan-500/40 transition-all">
              <Settings className="w-5 h-5 text-cyan-400" />
              <span className="text-sm">Settings</span>
            </button>
          </div>
        </Card>
      </div>
    </div>
  );
}
