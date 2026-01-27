# 多机器人集群调度系统 Dockerfile
# 生产环境优化版本

# 使用官方 Python 镜像 (slim 版本更轻量)
FROM python:3.11-slim

# 设置工作目录
WORKDIR /app

# 安装 curl (用于健康检查)
RUN apt-get update && \
    apt-get install -y --no-install-recommends curl && \
    rm -rf /var/lib/apt/lists/*

# 复制依赖文件
COPY requirements.txt .

# 安装 Python 依赖
RUN pip install --no-cache-dir -r requirements.txt

# 复制应用代码 (排除敏感文件)
COPY api/ ./api/
COPY src/ ./src/
COPY data/ ./data/

# 创建非 root 用户 (安全最佳实践)
RUN useradd -m -u 1000 appuser && \
    chown -R appuser:appuser /app
USER appuser

# 健康检查配置
HEALTHCHECK --interval=30s --timeout=10s --start-period=10s --retries=3 \
    CMD curl -f http://localhost:8000/system/system_status || exit 1

# 暴露端口
EXPOSE 8000

# 启动命令
CMD ["uvicorn", "api.main:app", "--host", "0.0.0.0", "--port", "8000"]