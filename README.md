# 🧭 Robot Dashboard Documentation

### 📌 Overview

This project is a full-stack robot dashboard featuring:

- **Frontend**: React + CSS
- **Backend**: FastAPI with WebSocket support
- **Teleop Integration**: Transitive Robotics component embedded in React
- **Session History**: Displays robot control logs via REST + WebSocket

---

### 🏗️ Project Architecture

```txt
┌────────────┐       ┌──────────────┐       ┌───────────────┐
│  Frontend  │ <---> │  Backend API │ <---> │  Redis / ROS  │
│ (React App)│       │ (FastAPI)    │       │  Bridges      │
└────────────┘       └──────────────┘       └───────────────┘
       ↑
       │
┌────────────────────────────┐
│ Transitive Web Component   │
│ (embedded in React)        │
└────────────────────────────┘
```

---

### 📁 Directory Structure

```txt
robot-dashboard/
├── frontend/              # React frontend
│   ├── src/               # Components, styles, Transitive embed
│   ├── public/            # Static assets
│   ├── .env.example       # Frontend env template
│   ├── package.json       # React dependencies
│   └── README.md
├── backend/               # FastAPI backend
│   ├── main.py            # API + WebSocket entry point
│   ├── ros1_bridge.py     # ROS1 integration (optional)
│   ├── ros2_bridge.py     # ROS2 integration (optional)
│   ├── requirements.txt   # Python dependencies
│   └── .gitignore
├── .gitignore             # Global ignore rules
└── README.md              # Project documentation
```

---

### 🚀 Startup Sequence (from GitHub clone)

#### 1. 📦 Clone the repo

```bash
git clone https://github.com/lucyy05/dragonfly-dashboard.git
cd robot-dashboard
```

#### 2. 🧪 Backend setup

```bash
cd backend

# Create and activate virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run backend
uvicorn main:app --reload
```

#### 3. 🎨 Frontend setup

```bash
cd ../frontend

# Copy and configure environment variables
cp .env.example .env
# Edit .env to point to your backend IP (e.g. http://localhost:8000)

# Install dependencies
npm install

# Start frontend
npm start
```

---

### 🔐 Environment Variables

#### `frontend/.env.example`

```env
REACT_APP_API_URL=http://localhost:8000
REACT_APP_WS_URL=ws://localhost:8000/ws/robot-data
```
