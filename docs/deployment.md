# Production Deployment Guide

This guide covers deploying the RAG Chatbot System to production environments with high availability, security, and performance considerations.

## 🏗️ Deployment Architecture

### Production Infrastructure
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Load Balancer │    │   Web Server    │    │   API Gateway   │
│   (Nginx/ALB)   │◄──►│   (Frontend)    │◄──►│   (Backend)     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                │                        │
                                ▼                        ▼
                       ┌─────────────────┐    ┌─────────────────┐
                       │   CDN           │    │   Database      │
                       │   (CloudFlare)  │    │   (Neon)        │
                       └─────────────────┘    └─────────────────┘
                                                        │
                                                        ▼
                                               ┌─────────────────┐
                                               │   Vector DB     │
                                               │   (Qdrant)      │
                                               └─────────────────┘
```

## 🚀 Deployment Options

### Option 1: Docker Compose (Recommended for Small-Medium Scale)

#### Prerequisites
- Docker 20.10+
- Docker Compose 2.0+
- Domain with SSL certificate
- 4GB+ RAM, 2+ CPU cores

#### Quick Deployment
```bash
# 1. Clone repository
git clone <repository-url>
cd rag-chatbot-system

# 2. Configure environment
cp .env.example .env.prod
# Edit .env.prod with production values

# 3. Deploy
docker-compose -f docker-compose.prod.yml up -d

# 4. Initialize database
docker-compose exec backend alembic upgrade head

# 5. Verify deployment
curl https://your-domain.com/api/health
```

#### Production Docker Compose Configuration
```yaml
# docker-compose.prod.yml
version: '3.8'

services:
  backend:
    build:
      context: ./book-backend
      dockerfile: Dockerfile.prod
    environment:
      - ENVIRONMENT=production
    env_file:
      - .env.prod
    restart: unless-stopped
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8000/health"]
      interval: 30s
      timeout: 10s
      retries: 3
    deploy:
      resources:
        limits:
          memory: 2G
          cpus: '1.0'
        reservations:
          memory: 1G
          cpus: '0.5'

  frontend:
    build:
      context: ./book-frontend
      dockerfile: Dockerfile.prod
    restart: unless-stopped
    depends_on:
      - backend
    deploy:
      resources:
        limits:
          memory: 512M
          cpus: '0.5'

  nginx:
    image: nginx:alpine
    ports:
      - "80:80"
      - "443:443"
    volumes:
      - ./nginx/nginx.conf:/etc/nginx/nginx.conf
      - ./ssl:/etc/nginx/ssl
    depends_on:
      - frontend
      - backend
    restart: unless-stopped

  redis:
    image: redis:alpine
    restart: unless-stopped
    command: redis-server --appendonly yes
    volumes:
      - redis_data:/data

volumes:
  redis_data:
```

### Option 2: Kubernetes (Recommended for Large Scale)

#### Prerequisites
- Kubernetes cluster (1.20+)
- kubectl configured
- Helm 3.0+
- Ingress controller

#### Kubernetes Deployment
```bash
# 1. Create namespace
kubectl create namespace rag-chatbot

# 2. Create secrets
kubectl create secret generic rag-secrets \
  --from-env-file=.env.prod \
  -n rag-chatbot

# 3. Deploy with Helm
helm install rag-chatbot ./helm/rag-chatbot \
  --namespace rag-chatbot \
  --values values.prod.yaml

# 4. Verify deployment
kubectl get pods -n rag-chatbot
kubectl get ingress -n rag-chatbot
```

#### Kubernetes Manifests
```yaml
# k8s/backend-deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: rag-backend
  namespace: rag-chatbot
spec:
  replicas: 3
  selector:
    matchLabels:
      app: rag-backend
  template:
    metadata:
      labels:
        app: rag-backend
    spec:
      containers:
      - name: backend
        image: your-registry/rag-backend:latest
        ports:
        - containerPort: 8000
        env:
        - name: ENVIRONMENT
          value: "production"
        envFrom:
        - secretRef:
            name: rag-secrets
        resources:
          requests:
            memory: "1Gi"
            cpu: "500m"
          limits:
            memory: "2Gi"
            cpu: "1000m"
        livenessProbe:
          httpGet:
            path: /health
            port: 8000
          initialDelaySeconds: 30
          periodSeconds: 10
        readinessProbe:
          httpGet:
            path: /health
            port: 8000
          initialDelaySeconds: 5
          periodSeconds: 5
---
apiVersion: v1
kind: Service
metadata:
  name: rag-backend-service
  namespace: rag-chatbot
spec:
  selector:
    app: rag-backend
  ports:
  - port: 8000
    targetPort: 8000
  type: ClusterIP
```

### Option 3: Cloud Platform Deployment

#### Vercel (Frontend) + Railway/Render (Backend)
```bash
# Frontend deployment to Vercel
cd book-frontend
vercel --prod

# Backend deployment to Railway
cd book-backend
railway login
railway link
railway up
```

#### AWS ECS/Fargate
```bash
# Build and push images
aws ecr get-login-password --region us-east-1 | docker login --username AWS --password-stdin <account>.dkr.ecr.us-east-1.amazonaws.com

docker build -t rag-backend ./book-backend
docker tag rag-backend:latest <account>.dkr.ecr.us-east-1.amazonaws.com/rag-backend:latest
docker push <account>.dkr.ecr.us-east-1.amazonaws.com/rag-backend:latest

# Deploy with ECS CLI or CloudFormation
ecs-cli compose --file docker-compose.aws.yml service up
```

## 🔧 Environment Configuration

### Production Environment Variables
```bash
# .env.prod
# Application
ENVIRONMENT=production
DEBUG=false
LOG_LEVEL=INFO

# Database
NEON_DATABASE_URL=postgresql://user:pass@host:5432/db?sslmode=require

# Vector Database
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-production-api-key

# AI Services
GEMINI_API_KEY=your-production-gemini-key

# Security
SECURITY_SALT=your-production-salt-32-chars-min
JWT_SECRET_KEY=your-production-jwt-secret-64-chars-min

# Performance
MAX_CONCURRENT_REQUESTS=100
REQUEST_TIMEOUT=30
RATE_LIMIT_REQUESTS=100
RATE_LIMIT_WINDOW=60

# Monitoring
SENTRY_DSN=https://your-sentry-dsn
DATADOG_API_KEY=your-datadog-key

# External Services
REDIS_URL=redis://redis:6379/0
ELASTICSEARCH_URL=https://your-elasticsearch-cluster
```

### Nginx Configuration
```nginx
# nginx/nginx.conf
events {
    worker_connections 1024;
}

http {
    upstream backend {
        server backend:8000;
    }

    upstream frontend {
        server frontend:3000;
    }

    # Rate limiting
    limit_req_zone $binary_remote_addr zone=api:10m rate=10r/s;
    limit_req_zone $binary_remote_addr zone=chat:10m rate=5r/s;

    server {
        listen 80;
        server_name your-domain.com;
        return 301 https://$server_name$request_uri;
    }

    server {
        listen 443 ssl http2;
        server_name your-domain.com;

        ssl_certificate /etc/nginx/ssl/cert.pem;
        ssl_certificate_key /etc/nginx/ssl/key.pem;
        ssl_protocols TLSv1.2 TLSv1.3;
        ssl_ciphers ECDHE-RSA-AES256-GCM-SHA512:DHE-RSA-AES256-GCM-SHA512;

        # Security headers
        add_header X-Frame-Options DENY;
        add_header X-Content-Type-Options nosniff;
        add_header X-XSS-Protection "1; mode=block";
        add_header Strict-Transport-Security "max-age=31536000; includeSubDomains";

        # Frontend
        location / {
            proxy_pass http://frontend;
            proxy_set_header Host $host;
            proxy_set_header X-Real-IP $remote_addr;
            proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
            proxy_set_header X-Forwarded-Proto $scheme;
        }

        # API endpoints
        location /api/ {
            limit_req zone=api burst=20 nodelay;
            proxy_pass http://backend;
            proxy_set_header Host $host;
            proxy_set_header X-Real-IP $remote_addr;
            proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
            proxy_set_header X-Forwarded-Proto $scheme;
            proxy_read_timeout 60s;
        }

        # Chat endpoints with stricter rate limiting
        location /api/chat/ {
            limit_req zone=chat burst=10 nodelay;
            proxy_pass http://backend;
            proxy_set_header Host $host;
            proxy_set_header X-Real-IP $remote_addr;
            proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
            proxy_set_header X-Forwarded-Proto $scheme;
            proxy_read_timeout 120s;
        }

        # Health check
        location /health {
            access_log off;
            proxy_pass http://backend/health;
        }
    }
}
```

## 🔒 Security Configuration

### SSL/TLS Setup
```bash
# Using Let's Encrypt with Certbot
certbot --nginx -d your-domain.com

# Or using custom certificates
mkdir -p ssl
cp your-cert.pem ssl/cert.pem
cp your-key.pem ssl/key.pem
chmod 600 ssl/key.pem
```

### Firewall Configuration
```bash
# UFW (Ubuntu)
ufw allow 22/tcp    # SSH
ufw allow 80/tcp    # HTTP
ufw allow 443/tcp   # HTTPS
ufw deny 8000/tcp   # Block direct backend access
ufw enable

# iptables
iptables -A INPUT -p tcp --dport 22 -j ACCEPT
iptables -A INPUT -p tcp --dport 80 -j ACCEPT
iptables -A INPUT -p tcp --dport 443 -j ACCEPT
iptables -A INPUT -p tcp --dport 8000 -j DROP
```

### Security Hardening
```bash
# Disable root login
sed -i 's/PermitRootLogin yes/PermitRootLogin no/' /etc/ssh/sshd_config

# Setup fail2ban
apt install fail2ban
systemctl enable fail2ban
systemctl start fail2ban

# Configure automatic security updates
apt install unattended-upgrades
dpkg-reconfigure -plow unattended-upgrades
```

## 📊 Monitoring and Logging

### Application Monitoring
```yaml
# docker-compose.monitoring.yml
version: '3.8'

services:
  prometheus:
    image: prom/prometheus
    ports:
      - "9090:9090"
    volumes:
      - ./monitoring/prometheus.yml:/etc/prometheus/prometheus.yml
      - prometheus_data:/prometheus

  grafana:
    image: grafana/grafana
    ports:
      - "3001:3000"
    environment:
      - GF_SECURITY_ADMIN_PASSWORD=admin
    volumes:
      - grafana_data:/var/lib/grafana
      - ./monitoring/grafana/dashboards:/etc/grafana/provisioning/dashboards

  loki:
    image: grafana/loki
    ports:
      - "3100:3100"
    volumes:
      - ./monitoring/loki.yml:/etc/loki/local-config.yaml

volumes:
  prometheus_data:
  grafana_data:
```

### Log Aggregation
```yaml
# Fluentd configuration
<source>
  @type forward
  port 24224
  bind 0.0.0.0
</source>

<match docker.**>
  @type elasticsearch
  host elasticsearch
  port 9200
  index_name docker-logs
  type_name _doc
</match>
```

## 🚀 Deployment Automation

### CI/CD Pipeline (GitHub Actions)
```yaml
# .github/workflows/deploy.yml
name: Deploy to Production

on:
  push:
    branches: [main]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Run tests
        run: |
          cd book-backend
          pip install -r requirements.txt
          pytest tests/

  build-and-deploy:
    needs: test
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      
      - name: Build Docker images
        run: |
          docker build -t rag-backend ./book-backend
          docker build -t rag-frontend ./book-frontend
      
      - name: Deploy to production
        run: |
          echo "${{ secrets.DEPLOY_KEY }}" | base64 -d > deploy_key
          chmod 600 deploy_key
          ssh -i deploy_key user@production-server "
            cd /opt/rag-chatbot &&
            git pull origin main &&
            docker-compose -f docker-compose.prod.yml up -d --build
          "
```

### Deployment Script
```bash
#!/bin/bash
# deploy.sh

set -e

ENVIRONMENT=${1:-staging}
VERSION=${2:-latest}

echo "Deploying RAG Chatbot System to $ENVIRONMENT..."

# Backup current deployment
if [ "$ENVIRONMENT" = "production" ]; then
    echo "Creating backup..."
    docker-compose -f docker-compose.prod.yml exec backend pg_dump $DATABASE_URL > backup_$(date +%Y%m%d_%H%M%S).sql
fi

# Pull latest changes
git pull origin main

# Build and deploy
docker-compose -f docker-compose.$ENVIRONMENT.yml pull
docker-compose -f docker-compose.$ENVIRONMENT.yml up -d --build

# Run migrations
docker-compose -f docker-compose.$ENVIRONMENT.yml exec backend alembic upgrade head

# Health check
echo "Waiting for services to start..."
sleep 30

HEALTH_CHECK=$(curl -s -o /dev/null -w "%{http_code}" http://localhost/api/health)
if [ "$HEALTH_CHECK" = "200" ]; then
    echo "✅ Deployment successful!"
else
    echo "❌ Health check failed. Rolling back..."
    docker-compose -f docker-compose.$ENVIRONMENT.yml down
    exit 1
fi

echo "🚀 RAG Chatbot System deployed successfully to $ENVIRONMENT"
```

## 📋 Post-Deployment Checklist

### Immediate Verification
- [ ] Health endpoints responding (200 OK)
- [ ] Database connectivity working
- [ ] Vector database accessible
- [ ] AI service integration functional
- [ ] SSL certificate valid
- [ ] DNS resolution correct

### Functional Testing
- [ ] Chat functionality working
- [ ] File upload processing
- [ ] Voice input/output
- [ ] Session management
- [ ] Error handling
- [ ] Rate limiting active

### Performance Testing
- [ ] Response times within SLA
- [ ] Load testing passed
- [ ] Memory usage normal
- [ ] CPU utilization acceptable
- [ ] Database performance optimal

### Security Verification
- [ ] Security headers present
- [ ] HTTPS enforced
- [ ] Input validation working
- [ ] Authentication functional
- [ ] Rate limiting effective
- [ ] Firewall rules active

### Monitoring Setup
- [ ] Application metrics collecting
- [ ] Log aggregation working
- [ ] Alerts configured
- [ ] Dashboards accessible
- [ ] Backup system active

## 🔄 Rollback Procedures

### Quick Rollback
```bash
# Rollback to previous version
docker-compose -f docker-compose.prod.yml down
git checkout HEAD~1
docker-compose -f docker-compose.prod.yml up -d

# Rollback database if needed
docker-compose -f docker-compose.prod.yml exec backend psql $DATABASE_URL < backup_latest.sql
```

### Blue-Green Deployment
```bash
# Deploy to green environment
docker-compose -f docker-compose.green.yml up -d

# Switch traffic
nginx -s reload  # Switch upstream to green

# Verify and cleanup blue
docker-compose -f docker-compose.blue.yml down
```

## 📞 Support and Maintenance

### Regular Maintenance Tasks
- **Daily**: Monitor logs and metrics
- **Weekly**: Security updates and patches
- **Monthly**: Performance optimization review
- **Quarterly**: Disaster recovery testing

### Emergency Contacts
- **On-call Engineer**: +1-xxx-xxx-xxxx
- **DevOps Team**: devops@company.com
- **Security Team**: security@company.com

### Escalation Procedures
1. **P0 (Critical)**: Immediate escalation to on-call
2. **P1 (High)**: Escalate within 1 hour
3. **P2 (Medium)**: Standard support queue
4. **P3 (Low)**: Next business day

---

## 🎯 Next Steps

After successful deployment:
1. Review the [Runbook](./runbook.md) for operational procedures
2. Set up [Monitoring](./monitoring.md) dashboards
3. Configure [Backup & Recovery](./backup_recovery.md) procedures
4. Review [Security](./security.md) best practices

For issues or questions, refer to the [Troubleshooting Guide](./runbook.md#troubleshooting) or contact the support team.