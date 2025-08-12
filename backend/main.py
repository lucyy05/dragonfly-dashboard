from fastapi import FastAPI, WebSocket, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import redis
import json
import asyncio
from typing import Dict, Any
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(title="Robot Dashboard API", version="1.0.0")

# CORS Configuration - to migitate CORS middleware concerns
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # React dev server
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic models for request/response
class RobotCommand(BaseModel):
    action: str
    parameters: Dict[str, Any] = {}

class RobotStatus(BaseModel):
    status: str
    timestamp: float
    data: Dict[str, Any] = {}

# Redis connection with error handling
try:
    redis_client = redis.Redis(host='localhost', port=6379, db=0, decode_responses=True)
    redis_client.ping()  # Test connection
    logger.info("✅ Redis connected successfully")
except redis.ConnectionError:
    logger.error("❌ Could not connect to Redis. Make sure Redis is running on localhost:6379")
    redis_client = None

@app.on_event("startup")
async def startup_event():
    logger.info("🚀 Robot Dashboard API starting up...")
    if redis_client:
        # Initialize some default values in Redis
        redis_client.set('robot_status', json.dumps({
            'status': 'idle', 
            'timestamp': asyncio.get_event_loop().time()
        }))

@app.on_event("shutdown")
async def shutdown_event():
    logger.info("🛑 Robot Dashboard API shutting down...")

@app.get("/")
async def root():
    return {
        "message": "Robot Dashboard API", 
        "version": "1.0.0",
        "redis_connected": redis_client is not None
    }

@app.get("/health")
async def health_check():
    redis_status = "connected" if redis_client and redis_client.ping() else "disconnected"
    return {
        "status": "healthy",
        "redis": redis_status,
        "timestamp": asyncio.get_event_loop().time()
    }

@app.get("/robot/status")
async def get_robot_status():
    if not redis_client:
        raise HTTPException(status_code=500, detail="Redis not available")
    
    try:
        status_data = redis_client.get('robot_status')
        if status_data:
            return json.loads(status_data)
        else:
            return {"status": "unknown", "timestamp": asyncio.get_event_loop().time()}
    except Exception as e:
        logger.error(f"Error getting robot status: {e}")
        raise HTTPException(status_code=500, detail="Error retrieving robot status")

@app.post("/robot/command")
async def send_robot_command(command: RobotCommand):
    if not redis_client:
        raise HTTPException(status_code=500, detail="Redis not available")
    
    try:
        # Store command with timestamp in Redis
        command_data = {
            "action": command.action,
            "parameters": command.parameters,
            "timestamp": asyncio.get_event_loop().time(),
            "status": "pending"
        }
        
        # Store in Redis for ROS2 bridge to pick up
        redis_client.set('robot_command', json.dumps(command_data))
        logger.info(f"Command stored in Redis: {command.action}")
        
        # Also store in command history
        history_key = f"command_history:{int(asyncio.get_event_loop().time())}"
        redis_client.set(history_key, json.dumps(command_data))
        redis_client.expire(history_key, 3600)  # Expire after 1 hour
        
        return {"message": "Command sent successfully", "command": command_data}
        
    except Exception as e:
        logger.error(f"Error sending command: {e}")
        raise HTTPException(status_code=500, detail="Error processing command")

@app.websocket("/ws/robot-data")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            # Get real-time data from Redis
            status = redis_client.get('robot_status')
            plc_data = redis_client.get('plc_registers')
            
            data = {
                "timestamp": asyncio.get_event_loop().time(),
                "robot_status": status,
                "plc_data": json.loads(plc_data) if plc_data else None
            }
            
            await websocket.send_json(data)
            await asyncio.sleep(0.1)  # 10Hz update rate
            
    except Exception as e:
        print(f"WebSocket error: {e}")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
