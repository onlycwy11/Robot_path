# Docker 部署文档

## 目录

- [部署架构](#部署架构)
- [环境准备](#环境准备)
- [快速部署](#快速部署)
- [配置说明](#配置说明)
- [常用命令](#常用命令)
- [生产环境部署](#生产环境部署)
- [故障排查](#故障排查)

---

## 部署架构

```
┌─────────────────────────────────────────────────────────────┐
│                    Docker Compose                            │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│   ┌─────────────────┐       ┌─────────────────┐            │
│   │  robot-scheduler│       │   mqtt-broker   │            │
│   │   (API 服务)    │ ←──── │   (Mosquitto)   │            │
│   │   Port: 8000    │       │   Port: 1883    │            │
│   └─────────────────┘       └─────────────────┘            │
│                                                              │
│   可选组件:                                                  │
│   ┌─────────────────┐                                       │
│   │   mqtt-dashboard│  ← MQTT 管理界面 (可选)               │
│   │   Port: 8080    │                                       │
│   └─────────────────┘                                       │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

---

## 环境准备

### 1. 安装 Docker

**Windows:**
```bash
# 下载并安装 Docker Desktop
# https://www.docker.com/products/docker-desktop

# 验证安装
docker --version
docker-compose --version
```

**Linux (Ubuntu/Debian):**
```bash
# 安装 Docker
sudo apt-get update
sudo apt-get install docker.io docker-compose

# 启动服务
sudo systemctl start docker
sudo systemctl enable docker

# 添加当前用户到 docker 组 (可选)
sudo usermod -aG docker $USER
```

### 2. 克隆项目

```bash
git clone <repository-url>
cd 4_28
```

---

## 快速部署

### 方式一：单容器部署 (仅 API)

```bash
# 构建镜像
docker build -t robot-scheduler:latest .

# 运行容器
docker run -d \
  --name robot-scheduler \
  -p 8000:8000 \
  --env-file .env \
  robot-scheduler:latest

# 查看日志
docker logs -f robot-scheduler
```

### 方式二：完整部署 (推荐)

使用 docker-compose 部署 API + MQTT Broker:

```bash
# 创建环境配置
cp .env.example .env
# 编辑 .env 文件，填写 MQTT 配置

# 启动所有服务
docker-compose up -d

# 查看服务状态
docker-compose ps

# 查看日志
docker-compose logs -f robot-scheduler
```

---

## 配置说明

### 环境变量 (.env)

```bash
# MQTT 配置
MQTT_HOST=mqtt-broker      # Docker 内部使用服务名
MQTT_PORT=1883
MQTT_USERNAME=robot
MQTT_PASSWORD=Robot#25
MQTT_CLIENT_ID=robot_scheduler_client
MQTT_TOPIC_PREFIX=pro
MQTT_STATUS_TOPIC=pro/robot/+/status_agg
MQTT_ENABLED=true
MQTT_MESSAGE_TIMEOUT=30

# API 配置 (可选)
API_HOST=0.0.0.0
API_PORT=8000
API_WORKERS=1              # 生产环境建议 2-4
```

### Docker Compose 配置

`docker-compose.yml` 文件说明：

```yaml
version: '3.8'

services:
  # API 服务
  robot-scheduler:
    build: .
    ports:
      - "8000:8000"
    env_file:
      - .env
    depends_on:
      - mqtt-broker
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8000/system/system_status"]
      interval: 30s
      timeout: 10s
      retries: 3

  # MQTT Broker (Mosquitto)
  mqtt-broker:
    image: eclipse-mosquitto:2.0
    ports:
      - "1883:1883"
      - "9001:9001"        # WebSocket 端口 (可选)
    volumes:
      - ./mqtt_config:/mosquitto/config
      - mqtt_data:/mosquitto/data
      - mqtt_log:/mosquitto/log
    restart: unless-stopped

volumes:
  mqtt_data:
  mqtt_log:
```

### Mosquitto 配置 (可选)

创建 `mqtt_config/mosquitto.conf`:

```conf
# 基础配置
listener 1883
allow_anonymous false
password_file /mosquitto/config/password.txt

# WebSocket 支持 (可选)
listener 9001
protocol websockets

# 日志配置
log_dest file /mosquitto/log/mosquitto.log
log_type all
```

创建用户密码文件:

```bash
# 进入容器创建用户
docker-compose exec mqtt-broker mosquitto_passwd -c /mosquitto/config/password.txt robot
# 输入密码: Robot#25
```

---

## 常用命令

### 服务管理

```bash
# 启动服务
docker-compose up -d

# 停止服务
docker-compose down

# 重启服务
docker-compose restart

# 重启单个服务
docker-compose restart robot-scheduler

# 查看状态
docker-compose ps

# 查看资源使用
docker stats
```

### 日志查看

```bash
# 查看所有日志
docker-compose logs

# 查看特定服务日志
docker-compose logs robot-scheduler

# 实时跟踪日志
docker-compose logs -f robot-scheduler

# 查看最近 100 行日志
docker-compose logs --tail=100 robot-scheduler
```

### 容器操作

```bash
# 进入容器
docker-compose exec robot-scheduler bash

# 执行命令
docker-compose exec robot-scheduler python -c "print('test')"

# 复制文件到容器
docker cp local_file.txt robot-scheduler:/app/

# 从容器复制文件
docker cp robot-scheduler:/app/logs/ ./local_logs/
```

### 镜像管理

```bash
# 重新构建镜像
docker-compose build --no-cache

# 查看镜像
docker images | grep robot-scheduler

# 删除旧镜像
docker image prune

# 推送镜像到仓库 (可选)
docker tag robot-scheduler:latest <registry>/robot-scheduler:latest
docker push <registry>/robot-scheduler:latest
```

---

## 生产环境部署

### 1. 优化 Dockerfile

生产环境建议使用多阶段构建:

```dockerfile
# 构建阶段
FROM python:3.11-slim as builder

WORKDIR /app

# 安装依赖到单独目录
COPY requirements.txt .
RUN pip install --no-cache-dir --target=/app/deps -r requirements.txt

# 运行阶段
FROM python:3.11-slim

WORKDIR /app

# 复制依赖
COPY --from=builder /app/deps /usr/local/lib/python3.11/site-packages

# 复制应用代码
COPY api/ ./api/
COPY src/ ./src/
COPY data/ ./data/

# 创建非 root 用户
RUN useradd -m -u 1000 appuser && \
    chown -R appuser:appuser /app
USER appuser

# 健康检查
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD curl -f http://localhost:8000/system/system_status || exit 1

# 启动命令
CMD ["uvicorn", "api.main:app", "--host", "0.0.0.0", "--port", "8000", "--workers", "2"]
```

### 2. 使用外部 MQTT Broker

生产环境建议使用专业的 MQTT 服务:

```yaml
# docker-compose.yml (生产配置)
services:
  robot-scheduler:
    build: .
    ports:
      - "8000:8000"
    env_file:
      - .env
    environment:
      - MQTT_HOST=your-mqtt-server.com  # 外部 MQTT
      - MQTT_PORT=1883
    restart: always
    deploy:
      resources:
        limits:
          cpus: '2'
          memory: 1G
        reservations:
          cpus: '0.5'
          memory: 256M
```

### 3. 反向代理配置 (Nginx)

```nginx
# nginx.conf
upstream robot_scheduler {
    server robot-scheduler:8000;
}

server {
    listen 80;
    server_name your-domain.com;

    location / {
        proxy_pass http://robot_scheduler;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }

    # WebSocket 支持 (MQTT over WebSocket)
    location /mqtt {
        proxy_pass http://mqtt-broker:9001;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
    }
}
```

### 4. 日志管理

```yaml
# docker-compose.yml 添加日志配置
services:
  robot-scheduler:
    logging:
      driver: "json-file"
      options:
        max-size: "10m"
        max-file: "3"
```

### 5. 数据持久化

```yaml
volumes:
  mqtt_data:
    driver: local
  mqtt_log:
    driver: local
  app_logs:
    driver: local
```

---

## 故障排查

### 常见问题

#### 1. 容器无法启动

```bash
# 检查日志
docker-compose logs robot-scheduler

# 检查配置文件
docker-compose config

# 检查端口占用
netstat -tlnp | grep 8000
```

#### 2. MQTT 连接失败

```bash
# 检查 MQTT 服务状态
docker-compose ps mqtt-broker

# 测试 MQTT 连接
docker-compose exec mqtt-broker mosquitto_sub -t "test" -v

# 检查网络连通性
docker-compose exec robot-scheduler ping mqtt-broker
```

#### 3. API 响应慢

```bash
# 查看资源使用
docker stats robot-scheduler

# 检查进程
docker-compose exec robot-scheduler ps aux

# 优化配置
# 增加 workers 数量
# 调整内存限制
```

#### 4. 健康检查失败

```bash
# 手动测试健康检查
docker-compose exec robot-scheduler curl http://localhost:8000/system/system_status

# 查看健康检查日志
docker inspect robot-scheduler | grep -A 10 "Health"
```

### 重置环境

```bash
# 停止并删除所有容器
docker-compose down

# 删除数据卷 (谨慎操作)
docker-compose down -v

# 重新构建
docker-compose build --no-cache

# 重新启动
docker-compose up -d
```

---

## 附录

### API 端点

| 端点 | 方法 | 说明 |
|------|------|------|
| `/docs` | GET | Swagger UI |
| `/redoc` | GET | ReDoc 文档 |
| `/system/system_status` | GET | 系统状态 |
| `/map/initialize` | POST | 初始化地图 |
| `/scheduler/schedule` | POST | 调度任务 |
| `/robots/status` | GET | 机器人状态 |

### 端口说明

| 端口 | 服务 | 说明 |
|------|------|------|
| 8000 | robot-scheduler | API 服务 |
| 1883 | mqtt-broker | MQTT TCP |
| 9001 | mqtt-broker | MQTT WebSocket |

### 相关文档

- [系统说明文档](./系统说明文档.md)
- [API 接口文档](http://localhost:8000/docs)
- [Docker 官方文档](https://docs.docker.com/)
- [Mosquitto 文档](https://mosquitto.org/documentation/)