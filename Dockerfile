# 使用官方Python镜像（ slim 版本更轻量）
FROM python:3.11-slim

# 设置工作目录（容器内的目录）
WORKDIR /app

# 复制当前目录所有文件到容器中
COPY . .

# 安装依赖（从 requirements.txt）
RUN pip install --no-cache-dir -r requirements.txt

# 启动API（Uvicorn）
CMD ["uvicorn", "api.main:app", "--host", "0.0.0.0", "--port", "8000"]