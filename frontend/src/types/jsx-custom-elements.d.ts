declare namespace JSX {
	interface IntrinsicElements {
		'transitive-capability': any;
	}
}

// Augment React JSX to recognize custom web components
import type * as React from 'react';
import { TransitiveCapability } from '@transitive-sdk/utils-web';

declare module 'react' {
	namespace JSX {
		interface IntrinsicElements {
			'TransitiveCapability': React.DetailedHTMLProps<React.HTMLAttributes<HTMLElement>, HTMLElement> & {
				jwt: string;
				control_rosVersion?: string;
				control_topic?: string;
				control_type?: string;
				count?: string;
				framerate?: string;
				height?: string;
				quantizer?: string;
				source?: string;
				streamtype?: string;
				timeout?: string;
				type?: string;
				width?: string;
			};
		}
	}
}


