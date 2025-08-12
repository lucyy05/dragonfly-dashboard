/// <reference types="react-scripts" />

// Global type declarations for custom elements
declare namespace JSX {
  interface IntrinsicElements {
    'remote-teleop-fleet': React.DetailedHTMLProps<React.HTMLAttributes<HTMLElement>, HTMLElement> & {
      id: string;
      host: string;
      ssl: string;
      jwt: string;
    };
  }
}/// <reference types="react-scripts" />
