// src/components/TeleopSection.tsx
import React from 'react';
import { TransitiveCapability } from '@transitive-sdk/utils-web';

type TeleopSectionProps = {
  className?: string;
};

export default function TeleopSection({ className = '' }: TeleopSectionProps) {
  const jwt = process.env.REACT_APP_TRANSITIVE_JWT as string; // put token in .env

  return (
    <div className={`teleop-container min-h-[600px] bg-gray-50 p-4 ${className}`}>
      <TransitiveCapability
        jwt={jwt}
        control_rosVersion="1"
        control_topic="/cmd_vel_nav"
        control_type="geometry_msgs/Twist"
        count="1"
        framerate="15/1"
        height="720"
        quantizer="25"
        source="/dev/video2"
        streamtype="video/x-raw"
        timeout="1800"
        type="v4l2src"
        width="1280"
      />
    </div>
  );
}