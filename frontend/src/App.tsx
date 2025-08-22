import React from 'react';
import RobotDashboard from './RobotDashboard';
import './App.css';

function App() {
  // const jwt = (process.env.REACT_APP_TRANSITIVE_JWT || '').trim();
  // if (!jwt || jwt.split('.').length !== 3) {
  //   return <div className="p-4 text-red-600">Invalid/missing teleop JWT. Check REACT_APP_TRANSITIVE_JWT and restart dev server.</div>;
  // }

  return (
    <div className="App">
      <RobotDashboard />
    </div>
  );
}


export default App;
