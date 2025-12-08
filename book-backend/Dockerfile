FROM python:3.10-slim

WORKDIR /app

# Copy requirements first
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy app code
COPY . .

# Expose port for Cloud Run
EXPOSE 8080

# Start command
CMD ["python", "main.py"]
