import { useState } from 'react';
import { Card } from './ui/card';
import { Badge } from './ui/badge';
import { Pause, Navigation, ZoomIn, Camera, Radar, Compass, Activity, MapPin } from 'lucide-react';
import { Progress } from './ui/progress';

export default function NavigationPanel() {
  const [speed, setSpeed] = useState(1.2);
  const [distanceCovered, setDistanceCovered] = useState(450);
  
  const telemetryData = {
    lidarStatus: 'Active',
    cameraFeed: 'Live',
    pitch: 2.3,
    roll: -1.1,
    yaw: 45.8,
    gpsPosition: '23.4°N, 12.8°E',
    slamConfidence: 92
  };
  
  const terrainType = 'Soft Dust';
  
  return (
    <div className="space-y-6">
      {/* Main Content Grid */}
      <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">
        {/* Map View - Takes up 3 columns */}
        <Card className="lg:col-span-3 bg-[#1F2833]/50 backdrop-blur-sm border-cyan-500/20 p-6">
          <div className="flex items-center justify-between mb-4">
            <div className="flex items-center gap-2">
              <Navigation className="w-5 h-5 text-cyan-400" />
              <h3 className="text-lg text-[#E5E5E5]">Real-Time Navigation Map</h3>
            </div>
            
            {/* Mini Control Overlay */}
            <div className="flex items-center gap-2">
              <button className="px-3 py-2 bg-cyan-500/20 hover:bg-cyan-500/30 text-cyan-400 rounded-lg border border-cyan-500/50 transition-colors text-sm flex items-center gap-2">
                <Pause className="w-4 h-4" />
                Pause
              </button>
              <button className="px-3 py-2 bg-cyan-500/20 hover:bg-cyan-500/30 text-cyan-400 rounded-lg border border-cyan-500/50 transition-colors text-sm flex items-center gap-2">
                <Compass className="w-4 h-4" />
                Recalculate
              </button>
              <button className="px-3 py-2 bg-cyan-500/20 hover:bg-cyan-500/30 text-cyan-400 rounded-lg border border-cyan-500/50 transition-colors text-sm flex items-center gap-2">
                <ZoomIn className="w-4 h-4" />
                Focus
              </button>
            </div>
          </div>
          
          {/* 3D Map Display */}
          <div className="relative bg-[#0B0C10] rounded-lg border border-cyan-500/20 h-[500px] overflow-hidden">
            {/* Grid background */}
            <div className="absolute inset-0" style={{
              backgroundImage: 'linear-gradient(rgba(0, 209, 255, 0.15) 1px, transparent 1px), linear-gradient(90deg, rgba(0, 209, 255, 0.15) 1px, transparent 1px)',
              backgroundSize: '50px 50px'
            }}>
              {/* Planned Path - Blue Line */}
              <svg className="absolute inset-0 w-full h-full">
                <path 
                  d="M 100 250 Q 200 200, 300 250 T 500 300 T 700 250" 
                  stroke="rgba(0, 150, 255, 0.6)" 
                  strokeWidth="3"
                  fill="none"
                  strokeDasharray="10,5"
                />
              </svg>
              
              {/* Actual Path - Green Line */}
              <svg className="absolute inset-0 w-full h-full">
                <path 
                  d="M 100 250 Q 200 205, 300 255 T 450 295" 
                  stroke="rgba(0, 255, 100, 0.8)" 
                  strokeWidth="3"
                  fill="none"
                />
              </svg>
              
              {/* Obstacles - Red Zones */}
              <div className="absolute left-[25%] top-[35%] w-16 h-16 bg-red-500/20 border-2 border-red-500 rounded-lg" />
              <div className="absolute left-[60%] top-[55%] w-20 h-12 bg-red-500/20 border-2 border-red-500 rounded-lg" />
              
              {/* LunaBot Position */}
              <div className="absolute left-[50%] top-[58%] -translate-x-1/2 -translate-y-1/2">
                <div className="relative">
                  {/* Direction indicator */}
                  <div className="absolute -top-8 left-1/2 -translate-x-1/2 w-0.5 h-6 bg-cyan-400" />
                  
                  {/* Bot marker */}
                  <div className="w-8 h-8 bg-cyan-400 rounded-full shadow-lg shadow-cyan-500/50 animate-pulse flex items-center justify-center">
                    <div className="w-4 h-4 bg-cyan-600 rounded-full" />
                  </div>
                  
                  {/* Label */}
                  <div className="absolute -bottom-8 left-1/2 -translate-x-1/2 whitespace-nowrap text-sm text-cyan-400">
                    LunaBot-01
                  </div>
                  
                  {/* Range circle */}
                  <div className="absolute inset-0 w-32 h-32 -translate-x-1/2 -translate-y-1/2 left-1/2 top-1/2 border-2 border-cyan-400/30 rounded-full" />
                </div>
              </div>
              
              {/* Landmarks */}
              <div className="absolute left-[80%] top-[20%]">
                <div className="w-6 h-6 border-2 border-green-400 rounded-full">
                  <div className="absolute -top-6 left-1/2 -translate-x-1/2 whitespace-nowrap text-xs text-green-400">
                    Base
                  </div>
                </div>
              </div>
              
              <div className="absolute left-[15%] top-[65%]">
                <div className="w-4 h-4 bg-yellow-400 rounded-sm">
                  <div className="absolute -top-6 left-1/2 -translate-x-1/2 whitespace-nowrap text-xs text-yellow-400">
                    Solar Farm
                  </div>
                </div>
              </div>
            </div>
            
            {/* Map Legend */}
            <div className="absolute bottom-4 right-4 bg-[#1F2833]/90 backdrop-blur-sm border border-cyan-500/20 rounded-lg p-3">
              <p className="text-xs text-cyan-400 mb-2">Map Legend</p>
              <div className="space-y-1.5">
                <div className="flex items-center gap-2">
                  <div className="w-3 h-3 bg-green-500 rounded-full" />
                  <span className="text-xs text-[#E5E5E5]">Safe Zone</span>
                </div>
                <div className="flex items-center gap-2">
                  <div className="w-3 h-3 bg-red-500 rounded-sm" />
                  <span className="text-xs text-[#E5E5E5]">Obstacle</span>
                </div>
                <div className="flex items-center gap-2">
                  <div className="w-3 h-0.5 bg-blue-400" />
                  <span className="text-xs text-[#E5E5E5]">Planned Route</span>
                </div>
                <div className="flex items-center gap-2">
                  <div className="w-3 h-3 bg-cyan-400 rounded-full" />
                  <span className="text-xs text-[#E5E5E5]">LunaBot Position</span>
                </div>
              </div>
            </div>
            
            {/* Speed and Distance Overlay */}
            <div className="absolute bottom-4 left-4 bg-[#1F2833]/90 backdrop-blur-sm border border-cyan-500/20 rounded-lg p-3">
              <div className="flex items-center gap-4">
                <div>
                  <p className="text-xs text-cyan-400/70">Speed</p>
                  <p className="text-lg text-cyan-400">{speed} m/s</p>
                </div>
                <div className="w-px h-8 bg-cyan-500/20" />
                <div>
                  <p className="text-xs text-cyan-400/70">Distance</p>
                  <p className="text-lg text-cyan-400">{distanceCovered} m</p>
                </div>
                <div className="w-px h-8 bg-cyan-500/20" />
                <div>
                  <p className="text-xs text-cyan-400/70">Terrain</p>
                  <p className="text-sm text-cyan-400">{terrainType}</p>
                </div>
              </div>
            </div>
          </div>
        </Card>
        
        {/* Telemetry Bar - Right Panel */}
        <Card className="bg-[#1F2833]/50 backdrop-blur-sm border-cyan-500/20 p-6">
          <div className="flex items-center gap-2 mb-4">
            <Activity className="w-5 h-5 text-cyan-400" />
            <h3 className="text-lg text-[#E5E5E5]">Telemetry</h3>
          </div>
          
          <div className="space-y-4">
            {/* LiDAR Status */}
            <div className="bg-cyan-500/5 rounded-lg p-3 border border-cyan-500/10">
              <div className="flex items-center justify-between mb-1">
                <span className="text-xs text-cyan-400/70">LiDAR Status</span>
                <Badge className="bg-green-500/20 text-green-400 border border-green-500/50">
                  {telemetryData.lidarStatus}
                </Badge>
              </div>
              <Radar className="w-6 h-6 text-cyan-400 mx-auto mt-2 animate-pulse" />
            </div>
            
            {/* Camera Feed */}
            <div className="bg-cyan-500/5 rounded-lg p-3 border border-cyan-500/10">
              <div className="flex items-center justify-between mb-1">
                <span className="text-xs text-cyan-400/70">Camera Feed</span>
                <Badge className="bg-green-500/20 text-green-400 border border-green-500/50">
                  {telemetryData.cameraFeed}
                </Badge>
              </div>
              <Camera className="w-6 h-6 text-cyan-400 mx-auto mt-2" />
            </div>
            
            {/* IMU Data */}
            <div className="bg-cyan-500/5 rounded-lg p-3 border border-cyan-500/10">
              <p className="text-xs text-cyan-400/70 mb-2">IMU Orientation</p>
              <div className="space-y-2">
                <div>
                  <div className="flex justify-between text-xs mb-1">
                    <span className="text-[#E5E5E5]">Pitch</span>
                    <span className="text-cyan-400">{telemetryData.pitch}°</span>
                  </div>
                  <Progress value={50 + telemetryData.pitch * 10} className="h-1.5 bg-cyan-950" />
                </div>
                <div>
                  <div className="flex justify-between text-xs mb-1">
                    <span className="text-[#E5E5E5]">Roll</span>
                    <span className="text-cyan-400">{telemetryData.roll}°</span>
                  </div>
                  <Progress value={50 + telemetryData.roll * 10} className="h-1.5 bg-cyan-950" />
                </div>
                <div>
                  <div className="flex justify-between text-xs mb-1">
                    <span className="text-[#E5E5E5]">Yaw</span>
                    <span className="text-cyan-400">{telemetryData.yaw}°</span>
                  </div>
                  <Progress value={telemetryData.yaw / 3.6} className="h-1.5 bg-cyan-950" />
                </div>
              </div>
            </div>
            
            {/* GPS Position */}
            <div className="bg-cyan-500/5 rounded-lg p-3 border border-cyan-500/10">
              <div className="flex items-center gap-2 mb-1">
                <MapPin className="w-4 h-4 text-cyan-400" />
                <span className="text-xs text-cyan-400/70">GPS Position</span>
              </div>
              <p className="text-sm text-cyan-400 mt-1">{telemetryData.gpsPosition}</p>
            </div>
            
            {/* SLAM Confidence */}
            <div className="bg-cyan-500/5 rounded-lg p-3 border border-cyan-500/10">
              <div className="flex justify-between text-xs mb-2">
                <span className="text-cyan-400/70">SLAM Confidence</span>
                <span className="text-cyan-400">{telemetryData.slamConfidence}%</span>
              </div>
              <Progress value={telemetryData.slamConfidence} className="h-2 bg-cyan-950" />
            </div>
          </div>
        </Card>
      </div>
      
      {/* Camera Feed Section */}
      <Card className="bg-[#1F2833]/50 backdrop-blur-sm border-cyan-500/20 p-6">
        <div className="flex items-center gap-2 mb-4">
          <Camera className="w-5 h-5 text-cyan-400" />
          <h3 className="text-lg text-[#E5E5E5]">Front Camera Feed</h3>
          <Badge className="bg-green-500/20 text-green-400 border border-green-500/50 ml-auto">
            Live
          </Badge>
        </div>
        
        <div className="relative bg-[#0B0C10] rounded-lg border border-cyan-500/20 h-64 overflow-hidden">
          {/* Simulated camera view with obstacle detection */}
          <div className="absolute inset-0 bg-gradient-to-b from-gray-800 to-gray-900 flex items-center justify-center">
            <p className="text-cyan-400/50">Camera Feed Simulation</p>
            
            {/* Obstacle detection boxes */}
            <div className="absolute left-[30%] top-[40%] w-24 h-32 border-2 border-red-500">
              <div className="absolute -top-6 left-0 bg-red-500/90 text-white px-2 py-1 rounded text-xs">
                Rock - 2.3m
              </div>
            </div>
            
            <div className="absolute left-[60%] top-[50%] w-20 h-24 border-2 border-yellow-500">
              <div className="absolute -top-6 left-0 bg-yellow-500/90 text-white px-2 py-1 rounded text-xs">
                Crater - 3.1m
              </div>
            </div>
          </div>
          
          {/* Crosshair overlay */}
          <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
            <div className="w-8 h-0.5 bg-cyan-400/50" />
            <div className="absolute w-0.5 h-8 bg-cyan-400/50" />
          </div>
        </div>
      </Card>
    </div>
  );
}
