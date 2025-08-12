// src/components/TeleopSection.tsx
import React, { useEffect, useState } from 'react';

interface TeleopSectionProps {
  className?: string;
}

// Type declaration for the custom web component
declare global {
  namespace JSX {
    interface IntrinsicElements {
      'remote-teleop-fleet': React.DetailedHTMLProps<React.HTMLAttributes<HTMLElement>, HTMLElement> & {
        id: string;
        host: string;
        ssl: string;
        jwt: string;
      };
    }
  }
}

const TeleopSection: React.FC<TeleopSectionProps> = ({ className = '' }) => {
  const [isScriptLoaded, setIsScriptLoaded] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    // Check if script is already loaded
    if (document.getElementById('transitive-teleop-script')) {
      setIsScriptLoaded(true);
      return;
    }

    // Load the Transitive Remote Teleop script
    const script = document.createElement('script');
    script.id = 'transitive-teleop-script';
    script.src = 'https://portal.transitiverobotics.com/running/@transitive-robotics/remote-teleop/dist/remote-teleop-fleet.js?userId=dragonfly&deviceId=_fleet';
    script.async = true;

    script.onload = () => {
      console.log('Transitive Remote Teleop script loaded successfully');
      setIsScriptLoaded(true);
      setError(null);
    };

    script.onerror = () => {
      console.error('Failed to load Transitive Remote Teleop script');
      setError('Failed to load remote teleop script');
      setIsScriptLoaded(false);
    };

    document.head.appendChild(script);

    // ⚠️ Do NOT remove the script on unmount.
    // The custom element must remain defined globally to avoid redefinition errors.
  }, []);

  if (error) {
    return (
      <div className={`bg-red-50 border border-red-200 rounded-lg p-4 ${className}`}>
        <h3 className="text-red-800 font-semibold mb-2">Remote Teleop Error</h3>
        <p className="text-red-600">{error}</p>
      </div>
    );
  }

  if (!isScriptLoaded) {
    return (
      <div className={`bg-gray-50 border rounded-lg p-8 ${className}`}>
        <div className="flex items-center justify-center space-x-3">
          <div className="animate-spin rounded-full h-6 w-6 border-b-2 border-blue-600"></div>
          <span className="text-gray-600">Loading Remote Teleop...</span>
        </div>
      </div>
    );
  }

  return (
    <div className={`bg-white border rounded-lg shadow-sm ${className}`}>
      <div className="p-4 border-b">
        <h3 className="text-lg font-semibold text-gray-800">Remote Teleop Control</h3>
        <p className="text-sm text-gray-600 mt-1">
          Use gamepad or on-screen controls to remotely operate the robot
        </p>
      </div>

      <div className="teleop-container min-h-[600px] bg-gray-50 p-4">
        {/* Transitive Remote Teleop Web Component */}
        {React.createElement('remote-teleop-fleet', {
          id: 'dragonfly',
          host: 'transitiverobotics.com',
          ssl: 'true',
          jwt: 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpZCI6ImRyYWdvbmZseSIsImRldmljZSI6Il9mbGVldCIsImNhcGFiaWxpdHkiOiJAdHJhbnNpdGl2ZS1yb2JvdGljcy9yZW1vdGUtdGVsZW9wIiwidmFsaWRpdHkiOjg2NDAwLCJpYXQiOjE3NTQ5OTE1Njh9.fcbnYR5Lu0KmOfPRCHkOV0IMxoR7gqE2KiVHvhDSguQ'
        })}
      </div>

      {/* 
      <div className="p-3 bg-gray-50 border-t">
        <div className="flex items-center space-x-2">
          <div className="w-2 h-2 bg-yellow-500 rounded-full animate-pulse"></div>
          <span className="text-sm text-gray-600">Initializing connection...</span>
        </div>
      </div> 
     */}

    </div>
  );
};

export default TeleopSection;
