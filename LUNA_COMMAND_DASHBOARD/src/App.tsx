import { useState } from 'react';
import { Tabs, TabsContent, TabsList, TabsTrigger } from './components/ui/tabs';
import MissionDashboard from './components/MissionDashboard';
import NavigationPanel from './components/NavigationPanel';
import CommandCenter from './components/CommandCenter';
import AlertReportPanel from './components/AlertReportPanel';
import { Home, Map, Gamepad2, Bell, Moon, Sun, Wifi } from 'lucide-react';

export default function App() {
  const [darkMode, setDarkMode] = useState(true);
  
  // Get current time
  const now = new Date();
  const lunarTime = new Date(now.getTime() + (24 * 60 * 60 * 1000 / 29.5)); // Simulated lunar time
  
  return (
    <div className={`min-h-screen ${darkMode ? 'bg-[#0B0C10]' : 'bg-gray-100'}`}>
      {/* Top Navigation Bar */}
      <div className="bg-[#0B0C10]/95 backdrop-blur-sm border-b border-cyan-500/20 px-6 py-4">
        <div className="flex items-center justify-between max-w-[1800px] mx-auto">
          {/* Logo & System Name */}
          <div className="flex items-center gap-3">
            <div className="w-10 h-10 bg-gradient-to-br from-cyan-400 to-blue-600 rounded-lg flex items-center justify-center">
              <Moon className="w-6 h-6 text-white" />
            </div>
            <div>
              <h1 className="text-[#E5E5E5] tracking-wide">LunaBot Mission Control</h1>
              <p className="text-xs text-cyan-400/70">Lunar Operations System v3.2</p>
            </div>
          </div>
          
          {/* Date & Time */}
          <div className="flex items-center gap-6">
            <div className="text-right">
              <p className="text-xs text-cyan-400/70">Lunar Time</p>
              <p className="text-sm text-[#E5E5E5]">{lunarTime.toLocaleTimeString()}</p>
            </div>
            <div className="text-right">
              <p className="text-xs text-cyan-400/70">UTC</p>
              <p className="text-sm text-[#E5E5E5]">{now.toLocaleTimeString()}</p>
            </div>
          </div>
          
          {/* Quick Actions */}
          <div className="flex items-center gap-4">
            <div className="flex items-center gap-2 px-3 py-2 bg-cyan-500/10 rounded-lg border border-cyan-500/20">
              <Wifi className="w-4 h-4 text-cyan-400" />
              <span className="text-xs text-[#E5E5E5]">Strong</span>
            </div>
            
            <button 
              onClick={() => setDarkMode(!darkMode)}
              className="p-2 hover:bg-cyan-500/10 rounded-lg transition-colors"
            >
              {darkMode ? <Sun className="w-5 h-5 text-cyan-400" /> : <Moon className="w-5 h-5 text-cyan-400" />}
            </button>
            
            <div className="w-10 h-10 bg-gradient-to-br from-cyan-500 to-blue-600 rounded-full flex items-center justify-center cursor-pointer hover:shadow-lg hover:shadow-cyan-500/50 transition-all">
              <span className="text-white">OP</span>
            </div>
          </div>
        </div>
      </div>
      
      {/* Main Content */}
      <div className="max-w-[1800px] mx-auto p-6">
        <Tabs defaultValue="dashboard" className="w-full">
          <TabsList className="grid w-full grid-cols-4 bg-[#1F2833]/50 backdrop-blur-sm border border-cyan-500/20 p-1 mb-6">
            <TabsTrigger 
              value="dashboard" 
              className="flex items-center gap-2 data-[state=active]:bg-cyan-500/20 data-[state=active]:text-cyan-400"
            >
              <Home className="w-4 h-4" />
              Mission Dashboard
            </TabsTrigger>
            <TabsTrigger 
              value="navigation"
              className="flex items-center gap-2 data-[state=active]:bg-cyan-500/20 data-[state=active]:text-cyan-400"
            >
              <Map className="w-4 h-4" />
              Navigation Panel
            </TabsTrigger>
            <TabsTrigger 
              value="command"
              className="flex items-center gap-2 data-[state=active]:bg-cyan-500/20 data-[state=active]:text-cyan-400"
            >
              <Gamepad2 className="w-4 h-4" />
              Command Center
            </TabsTrigger>
            <TabsTrigger 
              value="alerts"
              className="flex items-center gap-2 data-[state=active]:bg-cyan-500/20 data-[state=active]:text-cyan-400"
            >
              <Bell className="w-4 h-4" />
              Alerts & Reports
            </TabsTrigger>
          </TabsList>
          
          <TabsContent value="dashboard">
            <MissionDashboard />
          </TabsContent>
          
          <TabsContent value="navigation">
            <NavigationPanel />
          </TabsContent>
          
          <TabsContent value="command">
            <CommandCenter />
          </TabsContent>
          
          <TabsContent value="alerts">
            <AlertReportPanel />
          </TabsContent>
        </Tabs>
      </div>
    </div>
  );
}
