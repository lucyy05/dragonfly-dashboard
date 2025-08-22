import React, { useState, useEffect, useCallback } from 'react';
import axios from 'axios';

import TeleopSection from './components/TeleopSection';

// API configuration: derive WS URL from API_BASE or current host to avoid hardcoding
const API_BASE = process.env.REACT_APP_API_URL || `http://${window.location.hostname}:8000`;
const WS_URL = process.env.REACT_APP_WS_URL || `${API_BASE.replace(/^http(s?):/, 'ws$1:')}/ws/robot-data`;

interface RobotStatus {
  state: string;
  timestamp: number;
  node_status?: string;
  connections?: {
    redis_connected: boolean;
    ros2_node_active: boolean;
  };
}

interface RobotCommand {
  action: string;
  parameters: Record<string, any>;
}

const RobotDashboard: React.FC = () => {
  const [robotStatus, setRobotStatus] = useState<RobotStatus>({ state: 'Unknown', timestamp: 0 });
  const [isConnected, setIsConnected] = useState<boolean>(false);
  const [ws, setWs] = useState<WebSocket | null>(null);
  const [commands, setCommands] = useState<RobotCommand[]>([]);
  const [teleopMode, setTeleopMode] = useState<boolean>(false);
  const [connectionError, setConnectionError] = useState<string>('');

  // Keyboard controls for teleop
  const [activeKeys, setActiveKeys] = useState<Set<string>>(new Set());

  // WebSocket connection for real-time data

  useEffect(() => {
    let websocket: WebSocket | null = null;
    
    const connectWebSocket = () => {
      try {
        websocket = new WebSocket(WS_URL);
        
        websocket.onopen = () => {
          console.log('✅ WebSocket connected');
          setIsConnected(true);
          setConnectionError('');
        };
        
        websocket.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data);
            setRobotStatus({
              state: data.robot_status?.state || data.robot_status?.status || 'Unknown',
              timestamp: data.timestamp || Date.now() / 1000,
              node_status: data.robot_status?.node_status,
              connections: data.robot_status?.connections
            });
          } catch (e) {
            console.error('Error parsing WebSocket data:', e);
          }
        };
        
        websocket.onclose = () => {
          console.log('🔴 WebSocket disconnected');
          setIsConnected(false);
          // Try to reconnect after 3 seconds
          setTimeout(connectWebSocket, 3000);
        };
        
        websocket.onerror = (error) => {
          console.error('WebSocket error:', error);
          setIsConnected(false);
          setConnectionError('WebSocket connection failed');
        };
        
        setWs(websocket);
      } catch (error) {
        console.error('Failed to create WebSocket:', error);
        setConnectionError('Failed to connect to server');
        // Try to reconnect after 5 seconds
        setTimeout(connectWebSocket, 5000);
      }
    };

    connectWebSocket();
    
    return () => {
      if (websocket) {
        websocket.close();
      }
    };
  }, []); // Empty dependency array is now correct since we use local websocket variable


  // Send command to robot
  const sendCommand = useCallback(async (command: RobotCommand) => {
    try {
      const response = await axios.post(`${API_BASE}/robot/command`, command);
      console.log('✅ Command sent:', response.data);
      
      // Add to command history
      setCommands(prev => [...prev.slice(-9), { ...command, timestamp: Date.now() } as any]);
    } catch (error) {
      console.error('❌ Error sending command:', error);
      setConnectionError('Failed to send command. Check backend connection.');
    }
  }, []);

  // Keyboard event handlers for teleop
  useEffect(() => {
    if (!teleopMode) return;

    const handleKeyDown = (event: KeyboardEvent) => {
      event.preventDefault();
      const key = event.key.toLowerCase();
      
      if (!activeKeys.has(key)) {
        setActiveKeys(prev => new Set(prev).add(key));
        
        switch (key) {
          case 'w':
          case 'arrowup':
            sendCommand({ action: 'move_forward', parameters: { speed: 0.5 } });
            break;
          case 's':
          case 'arrowdown':
            sendCommand({ action: 'move_backward', parameters: { speed: 0.5 } });
            break;
          case 'a':
          case 'arrowleft':
            sendCommand({ action: 'turn_left', parameters: { speed: 0.3 } });
            break;
          case 'd':
          case 'arrowright':
            sendCommand({ action: 'turn_right', parameters: { speed: 0.3 } });
            break;
          case ' ':
            sendCommand({ action: 'stop', parameters: {} });
            break;
        }
      }
    };

    const handleKeyUp = (event: KeyboardEvent) => {
      const key = event.key.toLowerCase();
      setActiveKeys(prev => {
        const newSet = new Set(prev);
        newSet.delete(key);
        return newSet;
      });
      
      // Send stop command when key is released (for safety)
      if (['w', 's', 'a', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(key)) {
        sendCommand({ action: 'stop', parameters: {} });
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [teleopMode, activeKeys, sendCommand]);

  // Predefined robot commands
  const robotCommands = [
    { action: 'move_forward', parameters: { distance: 1.0 }, label: '⬆️ Forward', color: '#4CAF50' },
    { action: 'move_backward', parameters: { distance: 1.0 }, label: '⬇️ Backward', color: '#FF9800' },
    { action: 'turn_left', parameters: { angle: 90 }, label: '⬅️ Left', color: '#2196F3' },
    { action: 'turn_right', parameters: { angle: 90 }, label: '➡️ Right', color: '#2196F3' },
    { action: 'stop', parameters: {}, label: '🛑 STOP', color: '#F44336' },
    { action: 'home', parameters: {}, label: '🏠 Home', color: '#9C27B0' }
  ];

  return (
    <div style={{ 
      padding: '20px', 
      fontFamily: '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif',
      background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
      minHeight: '100vh',
      color: 'white'
    }}>
      <div style={{ maxWidth: '1200px', margin: '0 auto' }}>
        <h1 style={{ textAlign: 'center', marginBottom: '30px', fontSize: '2.5rem' }}>
          🤖 Robot Control Dashboard
        </h1>
        
        {/* Connection Status */}
        <div style={{ 
          display: 'grid', 
          gridTemplateColumns: 'repeat(auto-fit, minmax(300px, 1fr))', 
          gap: '20px',
          marginBottom: '30px'
        }}>
          <div style={{ 
            padding: '20px', 
            backgroundColor: isConnected ? 'rgba(0,255,0,0.2)' : 'rgba(255,0,0,0.2)',
            borderRadius: '10px',
            border: `2px solid ${isConnected ? '#00ff00' : '#ff0000'}`,
            backdropFilter: 'blur(10px)'
          }}>
            <h3>🌐 Connection Status</h3>
            <p><strong>WebSocket:</strong> {isConnected ? '🟢 Connected' : '🔴 Disconnected'}</p>
            <p><strong>Robot State:</strong> <span style={{ 
              color: robotStatus.state === 'idle' ? '#90EE90' : 
                     robotStatus.state.includes('error') ? '#FF6B6B' : '#FFD700' 
            }}>
              {robotStatus.state}
            </span></p>
            {robotStatus.connections && (
              <>
                <p><strong>Redis:</strong> {robotStatus.connections.redis_connected ? '🟢' : '🔴'}</p>
                <p><strong>ROS2:</strong> {robotStatus.connections.ros2_node_active ? '🟢' : '🔴'}</p>
              </>
            )}
            {connectionError && (
              <p style={{ color: '#FF6B6B', fontSize: '0.9rem' }}>⚠️ {connectionError}</p>
            )}
            <p style={{ fontSize: '0.8rem', opacity: 0.8 }}>
              Last Update: {new Date(robotStatus.timestamp * 1000).toLocaleTimeString()}
            </p>
          </div>

          {/* Teleop Mode Toggle */}
          <div style={{ 
            padding: '20px', 
            backgroundColor: 'rgba(255,255,255,0.1)',
            borderRadius: '10px',
            backdropFilter: 'blur(10px)'
          }}>
            <h3>🎮 Teleop Control</h3>
            <button
              onClick={() => setTeleopMode(!teleopMode)}
              style={{
                padding: '15px 30px',
                backgroundColor: teleopMode ? '#4CAF50' : '#757575',
                color: 'white',
                border: 'none',
                borderRadius: '25px',
                cursor: 'pointer',
                fontSize: '16px',
                fontWeight: 'bold',
                marginBottom: '15px',
                width: '100%'
              }}
            >
              {teleopMode ? '🔓 TELEOP ACTIVE' : '🔒 CLICK TO ACTIVATE TELEOP'}
            </button>
            {teleopMode && (
              <div style={{ fontSize: '0.9rem', opacity: 0.9 }}>
                <p><strong>Keyboard Controls:</strong></p>
                <p>W/↑ - Forward | S/↓ - Backward</p>
                <p>A/← - Left | D/→ - Right | Space - Stop</p>
                <p style={{ color: '#FFD700' }}>⚠️ Focus on this window for keyboard control</p>
              </div>
            )}
          </div>
        </div>

        {/* Manual Controls */}
        <div style={{ 
          backgroundColor: 'rgba(255,255,255,0.1)', 
          padding: '25px', 
          borderRadius: '15px',
          marginBottom: '25px',
          backdropFilter: 'blur(10px)'
        }}>
          <h3 style={{ marginBottom: '20px' }}>🕹️ Manual Controls</h3>
          <div style={{ 
            display: 'grid', 
            gridTemplateColumns: 'repeat(auto-fit, minmax(180px, 1fr))', 
            gap: '15px' 
          }}>
            {robotCommands.map((cmd, index) => (
              <button
                key={index}
                onClick={() => sendCommand(cmd)}
                disabled={!isConnected}
                style={{
                  padding: '15px 20px',
                  backgroundColor: isConnected ? cmd.color : '#666',
                  color: 'white',
                  border: 'none',
                  borderRadius: '10px',
                  cursor: isConnected ? 'pointer' : 'not-allowed',
                  fontSize: '16px',
                  fontWeight: 'bold',
                  transition: 'all 0.2s',
                  transform: 'scale(1)',
                  opacity: isConnected ? 1 : 0.5
                }}
                onMouseDown={(e) => isConnected && (e.currentTarget.style.transform = 'scale(0.95)')}
                onMouseUp={(e) => isConnected && (e.currentTarget.style.transform = 'scale(1)')}
                onMouseLeave={(e) => isConnected && (e.currentTarget.style.transform = 'scale(1)')}
              >
                {cmd.label}
              </button>
            ))}
          </div>
        </div>

        {/* Custom Command & Command History */}
        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '25px' }}>
          {/* Custom Command */}
          <div style={{ 
            backgroundColor: 'rgba(255,255,255,0.1)', 
            padding: '25px', 
            borderRadius: '15px',
            backdropFilter: 'blur(10px)'
          }}>
            <h3>⚙️ Custom Command</h3>
            <div style={{ display: 'flex', flexDirection: 'column', gap: '15px' }}>
              <input
                type="text"
                placeholder="Enter command (e.g., rotate_180)"
                id="customAction"
                disabled={!isConnected}
                style={{
                  padding: '12px',
                  borderRadius: '8px',
                  border: 'none',
                  fontSize: '16px',
                  backgroundColor: 'rgba(255,255,255,0.9)',
                  color: '#333'
                }}
              />
              <button
                onClick={() => {
                  const input = document.getElementById('customAction') as HTMLInputElement;
                  const action = input?.value.trim();
                  if (action) {
                    sendCommand({ action, parameters: {} });
                    input.value = '';
                  }
                }}
                disabled={!isConnected}
                style={{
                  padding: '12px 24px',
                  backgroundColor: isConnected ? '#2196F3' : '#666',
                  color: 'white',
                  border: 'none',
                  borderRadius: '8px',
                  cursor: isConnected ? 'pointer' : 'not-allowed',
                  fontSize: '16px',
                  fontWeight: 'bold'
                }}
              >
                Send Command
              </button>
            </div>
          </div>

          {/* Command History */}
          <div style={{ 
            backgroundColor: 'rgba(255,255,255,0.1)', 
            padding: '25px', 
            borderRadius: '15px',
            backdropFilter: 'blur(10px)'
          }}>
            <h3>📜 Command History</h3>
            <div style={{ 
              maxHeight: '200px', 
              overflowY: 'auto',
              backgroundColor: 'rgba(0,0,0,0.3)',
              padding: '15px',
              borderRadius: '8px'
            }}>
              {commands.length === 0 ? (
                <p style={{ color: '#ccc', textAlign: 'center' }}>No commands sent yet</p>
              ) : (
                commands.slice(-8).reverse().map((cmd, index) => (
                  <div key={index} style={{ 
                    padding: '8px 0', 
                    borderBottom: index < commands.slice(-8).length - 1 ? '1px solid rgba(255,255,255,0.1)' : 'none',
                    fontSize: '0.9rem'
                  }}>
                    <strong style={{ color: '#90EE90' }}>{cmd.action}</strong>
                    {Object.keys(cmd.parameters).length > 0 && (
                      <span style={{ color: '#FFD700', marginLeft: '10px' }}>
                        {JSON.stringify(cmd.parameters)}
                      </span>
                    )}
                  </div>
                ))
              )}
            </div>
          </div>
        </div>


	{/* ADD THE TELEOP SECTION HERE */}
	<TeleopSection className="mt-8" />

        {/* Emergency Stop */}
        <div style={{ 
          position: 'fixed', 
          bottom: '30px', 
          right: '30px',
          zIndex: 1000
        }}>
          <button
            onClick={() => sendCommand({ action: 'stop', parameters: {} })}
            style={{
              width: '80px',
              height: '80px',
              borderRadius: '50%',
              backgroundColor: '#FF4444',
              border: '4px solid white',
              color: 'white',
              fontSize: '24px',
              fontWeight: 'bold',
              cursor: 'pointer',
              boxShadow: '0 4px 20px rgba(255,68,68,0.4)',
              transition: 'all 0.2s'
            }}
            onMouseDown={(e) => e.currentTarget.style.transform = 'scale(0.9)'}
            onMouseUp={(e) => e.currentTarget.style.transform = 'scale(1)'}
            onMouseLeave={(e) => e.currentTarget.style.transform = 'scale(1)'}
          >
            STOP
          </button>
        </div>
      </div>
    </div>
  );
};

export default RobotDashboard;
