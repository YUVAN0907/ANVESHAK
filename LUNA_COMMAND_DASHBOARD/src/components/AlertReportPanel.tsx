import { useState } from 'react';
import { Card } from './ui/card';
import { Badge } from './ui/badge';
import { AlertTriangle, Wifi, Battery, Navigation, Activity, Download, Send, X, Filter } from 'lucide-react';
import { Tabs, TabsContent, TabsList, TabsTrigger } from './ui/tabs';

interface Alert {
  id: number;
  type: 'critical' | 'warning' | 'info';
  category: 'System' | 'Navigation' | 'Battery' | 'Communication' | 'AI';
  message: string;
  source: string;
  timestamp: string;
  action?: string;
}

export default function AlertReportPanel() {
  const [alerts, setAlerts] = useState<Alert[]>([
    {
      id: 1,
      type: 'warning',
      category: 'Battery',
      message: 'Battery below 20% on LunaBot-04',
      source: 'LunaBot-04',
      timestamp: '10:15 UTC',
      action: 'Return to base for recharge'
    },
    {
      id: 2,
      type: 'info',
      category: 'System',
      message: 'Data sync completed successfully',
      source: 'LunaBot-01',
      timestamp: '10:12 UTC'
    },
    {
      id: 3,
      type: 'critical',
      category: 'Navigation',
      message: 'Obstacle detected - navigation halted',
      source: 'LunaBot-03',
      timestamp: '10:08 UTC',
      action: 'Switch to manual mode'
    },
    {
      id: 4,
      type: 'info',
      category: 'Communication',
      message: 'Signal strength improved to Strong',
      source: 'LunaBot-02',
      timestamp: '10:05 UTC'
    },
    {
      id: 5,
      type: 'warning',
      category: 'AI',
      message: 'Path recalculation required',
      source: 'LunaBot-01',
      timestamp: '10:02 UTC',
      action: 'Approve new route'
    }
  ]);
  
  const [activeFilter, setActiveFilter] = useState<string>('all');
  
  const missionReport = {
    name: 'Soil Analysis 02',
    duration: '3h 20m',
    pathCovered: '2.8 km',
    powerUsed: 35,
    obstaclesAvoided: 12,
    dataCaptured: '1.2 GB images',
    outcome: 'Successful'
  };
  
  const aiInsights = [
    'Optimal time to recharge: 14:00 lunar time',
    'Next safe route suggestion available',
    'Dust accumulation risk: High - recommend cleaning in 6 hours'
  ];
  
  const getAlertColor = (type: string) => {
    switch(type) {
      case 'critical': return 'bg-red-500/20 border-red-500 text-red-400';
      case 'warning': return 'bg-yellow-500/20 border-yellow-500 text-yellow-400';
      case 'info': return 'bg-cyan-500/20 border-cyan-500 text-cyan-400';
      default: return 'bg-gray-500/20 border-gray-500 text-gray-400';
    }
  };
  
  const getCategoryIcon = (category: string) => {
    switch(category) {
      case 'System': return Activity;
      case 'Navigation': return Navigation;
      case 'Battery': return Battery;
      case 'Communication': return Wifi;
      case 'AI': return Activity;
      default: return AlertTriangle;
    }
  };
  
  const dismissAlert = (id: number) => {
    setAlerts(alerts.filter(alert => alert.id !== id));
  };
  
  const filteredAlerts = activeFilter === 'all' 
    ? alerts 
    : alerts.filter(alert => alert.category.toLowerCase() === activeFilter);

  return (
    <div className="space-y-6">
      <Tabs defaultValue="alerts" className="w-full">
        <TabsList className="grid w-full grid-cols-2 bg-[#1F2833]/50 backdrop-blur-sm border border-cyan-500/20 p-1">
          <TabsTrigger 
            value="alerts"
            className="data-[state=active]:bg-cyan-500/20 data-[state=active]:text-cyan-400"
          >
            Active Alerts
          </TabsTrigger>
          <TabsTrigger 
            value="reports"
            className="data-[state=active]:bg-cyan-500/20 data-[state=active]:text-cyan-400"
          >
            Mission Reports
          </TabsTrigger>
        </TabsList>
        
        <TabsContent value="alerts" className="mt-6">
          <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">
            {/* Notifications List */}
            <Card className="lg:col-span-3 bg-[#1F2833]/50 backdrop-blur-sm border-cyan-500/20 p-6">
              <div className="flex items-center justify-between mb-4">
                <h3 className="text-lg text-[#E5E5E5] flex items-center gap-2">
                  <AlertTriangle className="w-5 h-5 text-cyan-400" />
                  Active Notifications
                </h3>
                <Badge className="bg-cyan-500/20 text-cyan-400 border border-cyan-500/50">
                  {filteredAlerts.length} Active
                </Badge>
              </div>
              
              {/* Alert Filters */}
              <div className="flex items-center gap-2 mb-4 flex-wrap">
                <button 
                  onClick={() => setActiveFilter('all')}
                  className={`px-3 py-1.5 rounded-lg text-sm transition-colors flex items-center gap-2 ${activeFilter === 'all' ? 'bg-cyan-500/20 text-cyan-400 border border-cyan-500/50' : 'bg-cyan-500/5 text-[#E5E5E5] border border-cyan-500/10'}`}
                >
                  <Filter className="w-4 h-4" />
                  All
                </button>
                <button 
                  onClick={() => setActiveFilter('system')}
                  className={`px-3 py-1.5 rounded-lg text-sm transition-colors ${activeFilter === 'system' ? 'bg-cyan-500/20 text-cyan-400 border border-cyan-500/50' : 'bg-cyan-500/5 text-[#E5E5E5] border border-cyan-500/10'}`}
                >
                  System
                </button>
                <button 
                  onClick={() => setActiveFilter('navigation')}
                  className={`px-3 py-1.5 rounded-lg text-sm transition-colors ${activeFilter === 'navigation' ? 'bg-cyan-500/20 text-cyan-400 border border-cyan-500/50' : 'bg-cyan-500/5 text-[#E5E5E5] border border-cyan-500/10'}`}
                >
                  Navigation
                </button>
                <button 
                  onClick={() => setActiveFilter('battery')}
                  className={`px-3 py-1.5 rounded-lg text-sm transition-colors ${activeFilter === 'battery' ? 'bg-cyan-500/20 text-cyan-400 border border-cyan-500/50' : 'bg-cyan-500/5 text-[#E5E5E5] border border-cyan-500/10'}`}
                >
                  Battery
                </button>
                <button 
                  onClick={() => setActiveFilter('communication')}
                  className={`px-3 py-1.5 rounded-lg text-sm transition-colors ${activeFilter === 'communication' ? 'bg-cyan-500/20 text-cyan-400 border border-cyan-500/50' : 'bg-cyan-500/5 text-[#E5E5E5] border border-cyan-500/10'}`}
                >
                  Communication
                </button>
                <button 
                  onClick={() => setActiveFilter('ai')}
                  className={`px-3 py-1.5 rounded-lg text-sm transition-colors ${activeFilter === 'ai' ? 'bg-cyan-500/20 text-cyan-400 border border-cyan-500/50' : 'bg-cyan-500/5 text-[#E5E5E5] border border-cyan-500/10'}`}
                >
                  AI / Control
                </button>
              </div>
              
              {/* Alerts */}
              <div className="space-y-3">
                {filteredAlerts.map((alert) => {
                  const Icon = getCategoryIcon(alert.category);
                  return (
                    <div 
                      key={alert.id}
                      className={`border-l-4 rounded-lg p-4 ${getAlertColor(alert.type)} bg-opacity-5 backdrop-blur-sm relative`}
                    >
                      <button 
                        onClick={() => dismissAlert(alert.id)}
                        className="absolute top-2 right-2 p-1 hover:bg-white/10 rounded transition-colors"
                      >
                        <X className="w-4 h-4" />
                      </button>
                      
                      <div className="flex items-start gap-3 pr-8">
                        <Icon className="w-5 h-5 mt-0.5" />
                        <div className="flex-1">
                          <div className="flex items-center gap-2 mb-1">
                            <Badge className={`${getAlertColor(alert.type)} border text-xs`}>
                              {alert.type.toUpperCase()}
                            </Badge>
                            <Badge className="bg-cyan-500/20 text-cyan-400 border border-cyan-500/50 text-xs">
                              {alert.category}
                            </Badge>
                          </div>
                          
                          <p className="text-[#E5E5E5] mb-1">{alert.message}</p>
                          
                          <div className="flex items-center justify-between mt-2">
                            <div className="flex items-center gap-3 text-xs">
                              <span className="text-cyan-400/70">Source: {alert.source}</span>
                              <span className="text-cyan-400/70">Time: {alert.timestamp}</span>
                            </div>
                          </div>
                          
                          {alert.action && (
                            <div className="mt-2 p-2 bg-cyan-500/10 rounded border border-cyan-500/20">
                              <p className="text-xs text-cyan-400">
                                Suggested Action: {alert.action}
                              </p>
                            </div>
                          )}
                        </div>
                      </div>
                    </div>
                  );
                })}
              </div>
            </Card>
            
            {/* AI Insights Panel */}
            <Card className="bg-[#1F2833]/50 backdrop-blur-sm border-cyan-500/20 p-6">
              <h3 className="text-lg text-[#E5E5E5] mb-4 flex items-center gap-2">
                <Activity className="w-5 h-5 text-cyan-400" />
                AI Insights
              </h3>
              
              <div className="space-y-3">
                {aiInsights.map((insight, index) => (
                  <div 
                    key={index}
                    className="bg-cyan-500/5 rounded-lg p-3 border border-cyan-500/20"
                  >
                    <p className="text-sm text-[#E5E5E5]">{insight}</p>
                  </div>
                ))}
              </div>
            </Card>
          </div>
        </TabsContent>
        
        <TabsContent value="reports" className="mt-6">
          <div className="space-y-6">
            {/* Mission Summary Report */}
            <Card className="bg-[#1F2833]/50 backdrop-blur-sm border-cyan-500/20 p-6">
              <div className="flex items-center justify-between mb-6">
                <h3 className="text-xl text-[#E5E5E5]">Mission Summary Report</h3>
                <div className="flex items-center gap-2">
                  <button className="px-4 py-2 bg-cyan-500/20 hover:bg-cyan-500/30 text-cyan-400 rounded-lg border border-cyan-500/50 transition-colors text-sm flex items-center gap-2">
                    <Download className="w-4 h-4" />
                    Download PDF
                  </button>
                  <button className="px-4 py-2 bg-cyan-500/20 hover:bg-cyan-500/30 text-cyan-400 rounded-lg border border-cyan-500/50 transition-colors text-sm flex items-center gap-2">
                    <Send className="w-4 h-4" />
                    Send to Earth Base
                  </button>
                </div>
              </div>
              
              <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4 mb-6">
                <div className="bg-cyan-500/5 rounded-lg p-4 border border-cyan-500/10">
                  <p className="text-xs text-cyan-400/70 mb-1">Mission Name</p>
                  <p className="text-lg text-[#E5E5E5]">{missionReport.name}</p>
                </div>
                
                <div className="bg-cyan-500/5 rounded-lg p-4 border border-cyan-500/10">
                  <p className="text-xs text-cyan-400/70 mb-1">Duration</p>
                  <p className="text-lg text-cyan-400">{missionReport.duration}</p>
                </div>
                
                <div className="bg-cyan-500/5 rounded-lg p-4 border border-cyan-500/10">
                  <p className="text-xs text-cyan-400/70 mb-1">Path Covered</p>
                  <p className="text-lg text-cyan-400">{missionReport.pathCovered}</p>
                </div>
                
                <div className="bg-cyan-500/5 rounded-lg p-4 border border-cyan-500/10">
                  <p className="text-xs text-cyan-400/70 mb-1">Outcome</p>
                  <Badge className="bg-green-500/20 text-green-400 border border-green-500/50">
                    ✅ {missionReport.outcome}
                  </Badge>
                </div>
              </div>
              
              <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
                <div className="bg-cyan-500/5 rounded-lg p-4 border border-cyan-500/10">
                  <p className="text-xs text-cyan-400/70 mb-1">Power Used</p>
                  <p className="text-2xl text-cyan-400">{missionReport.powerUsed}%</p>
                  <div className="w-full bg-cyan-950 rounded-full h-2 mt-2">
                    <div 
                      className="bg-gradient-to-r from-cyan-400 to-green-400 h-2 rounded-full"
                      style={{ width: `${missionReport.powerUsed}%` }}
                    />
                  </div>
                </div>
                
                <div className="bg-cyan-500/5 rounded-lg p-4 border border-cyan-500/10">
                  <p className="text-xs text-cyan-400/70 mb-1">Obstacles Avoided</p>
                  <p className="text-2xl text-cyan-400">{missionReport.obstaclesAvoided}</p>
                  <p className="text-xs text-green-400 mt-1">Autonomous navigation successful</p>
                </div>
                
                <div className="bg-cyan-500/5 rounded-lg p-4 border border-cyan-500/10">
                  <p className="text-xs text-cyan-400/70 mb-1">Data Captured</p>
                  <p className="text-2xl text-cyan-400">{missionReport.dataCaptured}</p>
                  <p className="text-xs text-cyan-400/70 mt-1">Successfully uploaded to base</p>
                </div>
              </div>
            </Card>
            
            {/* Log History */}
            <Card className="bg-[#1F2833]/50 backdrop-blur-sm border-cyan-500/20 p-6">
              <h3 className="text-lg text-[#E5E5E5] mb-4">Mission Log History</h3>
              
              <div className="overflow-x-auto">
                <table className="w-full">
                  <thead>
                    <tr className="border-b border-cyan-500/20">
                      <th className="text-left text-xs text-cyan-400/70 pb-3">Timestamp</th>
                      <th className="text-left text-xs text-cyan-400/70 pb-3">Event Type</th>
                      <th className="text-left text-xs text-cyan-400/70 pb-3">Description</th>
                      <th className="text-left text-xs text-cyan-400/70 pb-3">Source</th>
                      <th className="text-left text-xs text-cyan-400/70 pb-3">Status</th>
                    </tr>
                  </thead>
                  <tbody>
                    <tr className="border-b border-cyan-500/10">
                      <td className="py-3 text-sm text-[#E5E5E5]">09:45 UTC</td>
                      <td className="py-3">
                        <Badge className="bg-cyan-500/20 text-cyan-400 border border-cyan-500/50">
                          Mission Start
                        </Badge>
                      </td>
                      <td className="py-3 text-sm text-[#E5E5E5]">Mission initialized successfully</td>
                      <td className="py-3 text-sm text-cyan-400">LunaBot-01</td>
                      <td className="py-3 text-sm text-green-400">Success</td>
                    </tr>
                    <tr className="border-b border-cyan-500/10">
                      <td className="py-3 text-sm text-[#E5E5E5]">10:12 UTC</td>
                      <td className="py-3">
                        <Badge className="bg-cyan-500/20 text-cyan-400 border border-cyan-500/50">
                          Navigation
                        </Badge>
                      </td>
                      <td className="py-3 text-sm text-[#E5E5E5]">Obstacle detected and avoided</td>
                      <td className="py-3 text-sm text-cyan-400">LunaBot-01</td>
                      <td className="py-3 text-sm text-green-400">Success</td>
                    </tr>
                    <tr className="border-b border-cyan-500/10">
                      <td className="py-3 text-sm text-[#E5E5E5]">11:30 UTC</td>
                      <td className="py-3">
                        <Badge className="bg-cyan-500/20 text-cyan-400 border border-cyan-500/50">
                          Data Collection
                        </Badge>
                      </td>
                      <td className="py-3 text-sm text-[#E5E5E5]">Soil samples collected and analyzed</td>
                      <td className="py-3 text-sm text-cyan-400">LunaBot-01</td>
                      <td className="py-3 text-sm text-green-400">Success</td>
                    </tr>
                    <tr className="border-b border-cyan-500/10">
                      <td className="py-3 text-sm text-[#E5E5E5]">13:05 UTC</td>
                      <td className="py-3">
                        <Badge className="bg-cyan-500/20 text-cyan-400 border border-cyan-500/50">
                          Mission Complete
                        </Badge>
                      </td>
                      <td className="py-3 text-sm text-[#E5E5E5]">Mission completed ahead of schedule</td>
                      <td className="py-3 text-sm text-cyan-400">LunaBot-01</td>
                      <td className="py-3 text-sm text-green-400">Success</td>
                    </tr>
                  </tbody>
                </table>
              </div>
            </Card>
          </div>
        </TabsContent>
      </Tabs>
    </div>
  );
}
