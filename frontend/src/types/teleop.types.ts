// src/types/teleop.types.ts
import React from 'react';

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

// This export makes it a module
export {};
