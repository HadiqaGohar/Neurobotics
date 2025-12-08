# RAG Chatbot System Runbook

This runbook provides operational procedures, troubleshooting guides, and emergency response protocols for the RAG Chatbot System.

## 🚨 Emergency Response

### Incident Severity Levels

#### P0 - Critical (Response: Immediate)
- **System completely down**
- **Data loss or corruption**
- **Security breach**
- **Complete service unavailability**

**Response Actions:**
1. Page on-call engineer immediately
2. Create incident channel (#incident-YYYYMMDD-HHMM)
3. Notify stakeholders within 15 minutes
4. Begin immediate investigation and mitigation

#### P1 - High (Response: 1 hour)
- **Major feature broken**
- **Significant performance degradation**
- **Partial service unavailability**
- **Authentication/authorization issues**

#### P2 - Medium (Response: 4 hours)
- **Minor feature issues**
- **Non-critical performance issues**
- **Cosmetic bugs affecting user experience**

#### P3 - Low (Response: Next business day)
- **Enhancement requests**
- **Documentation updates**
- **Minor improvements**

### Emergency Contacts

```
On-Call Rotation: +1-xxx-xxx-xxxx
DevOps Team: devops@company.com
Security Team: security@company.com
Management: management@company.com

Escalation Matrix:
├── L1: On-call Engineer (0-30 min)
├── L2: Senior Engineer (30-60 min)
├── L3: Engineering Manager (1-2 hours)
└── L4: CTO (2+ hours)
```

## 🔍 System Health Monitoring

### Health Check Endpoints

#### Primary Health Check
```bash
curl -f https://your-domain.com/api/health
```

Expected Response:
```json
{
  "status": "healthy",
  "environment": "production",
  "version": "1.0.0",
  "timestamp": "2024-01-01T00:00:00Z"
}
```

#### Detailed Health Check
```bash
curl -f https://your-domain.com/api/health/detailed
```

Expected Response:
```json
{
  "status": "healthy",
  "services": {
    "database": "healthy",
    "vector_db": "healthy",
    "ai_service": "healthy",
    "cache": "healthy"
  },
  "metrics": {
    "response_time": 0.123,
    "memory_usage": 0.65,
    "cpu_usage": 0.45
  }
}
```

### Key Metrics to Monitor

#### Application Metrics
- **Response Time**: < 2 seconds (95th percentile)
- **Error Rate**: < 1%
- **Throughput**: > 50 requests/second
- **Availability**: > 99.9%

#### System Metrics
- **CPU Usage**: < 80%
- **Memory Usage**: < 85%
- **Disk Usage**: < 90%
- **Network I/O**: Monitor for anomalies

#### Database Metrics
- **Connection Pool**: < 80% utilization
- **Query Performance**: < 100ms average
- **Replication Lag**: < 1 second
- **Lock Waits**: < 5% of queries

### Monitoring Commands

#### Check Service Status
```bash
# Docker Compose
docker-compose ps

# Kubernetes
kubectl get pods -n rag-chatbot
kubectl get services -n rag-chatbot
kubectl get ingress -n rag-chatbot
```

#### Check Logs
```bash
# Docker Compose
docker-compose logs -f backend
docker-compose logs -f frontend
docker-compose logs --tail=100 nginx

# Kubernetes
kubectl logs -f deployment/rag-backend -n rag-chatbot
kubectl logs -f deployment/rag-frontend -n rag-chatbot
```

#### Check Resource Usage
```bash
# System resources
htop
df -h
free -h
iostat -x 1

# Docker resources
docker stats
docker system df

# Kubernetes resources
kubectl top nodes
kubectl top pods -n rag-chatbot
```

## 🛠️ Common Operations

### Deployment Operations

#### Deploy New Version
```bash
# 1. Backup current state
./scripts/backup.sh

# 2. Deploy new version
./deploy.sh production v1.2.0

# 3. Verify deployment
curl -f https://your-domain.com/api/health
./scripts/smoke-test.sh

# 4. Monitor for issues
tail -f /var/log/rag-chatbot/app.log
```

#### Rollback Deployment
```bash
# Quick rollback
./scripts/rollback.sh

# Manual rollback
git checkout v1.1.0
docker-compose -f docker-compose.prod.yml up -d --build

# Database rollback (if needed)
./scripts/restore-db.sh backup_20240101_120000.sql
```

### Database Operations

#### Database Backup
```bash
# Manual backup
pg_dump $DATABASE_URL > backup_$(date +%Y%m%d_%H%M%S).sql

# Automated backup script
./scripts/backup-db.sh
```

#### Database Restore
```bash
# Restore from backup
psql $DATABASE_URL < backup_20240101_120000.sql

# Verify restore
psql $DATABASE_URL -c "SELECT COUNT(*) FROM sessions;"
```

#### Database Migrations
```bash
# Check migration status
docker-compose exec backend alembic current

# Apply migrations
docker-compose exec backend alembic upgrade head

# Rollback migration
docker-compose exec backend alembic downgrade -1
```

### Cache Operations

#### Clear Cache
```bash
# Redis cache
docker-compose exec redis redis-cli FLUSHALL

# Application cache
curl -X POST https://your-domain.com/api/admin/cache/clear \
  -H "Authorization: Bearer $ADMIN_TOKEN"
```

#### Cache Statistics
```bash
# Redis stats
docker-compose exec redis redis-cli INFO stats

# Application cache stats
curl https://your-domain.com/api/admin/cache/stats \
  -H "Authorization: Bearer $ADMIN_TOKEN"
```

### Log Management

#### Log Rotation
```bash
# Manual log rotation
logrotate -f /etc/logrotate.d/rag-chatbot

# Check log sizes
du -sh /var/log/rag-chatbot/*
```

#### Log Analysis
```bash
# Error analysis
grep -i error /var/log/rag-chatbot/app.log | tail -20

# Performance analysis
grep "response_time" /var/log/rag-chatbot/app.log | awk '{print $5}' | sort -n

# User activity
grep "chat_request" /var/log/rag-chatbot/app.log | wc -l
```

## 🔧 Troubleshooting Guide

### Common Issues and Solutions

#### Issue: High Response Times

**Symptoms:**
- API responses > 5 seconds
- User complaints about slow chat
- High CPU/memory usage

**Investigation:**
```bash
# Check system resources
htop
iostat -x 1

# Check database performance
docker-compose exec backend psql $DATABASE_URL -c "
SELECT query, mean_exec_time, calls 
FROM pg_stat_statements 
ORDER BY mean_exec_time DESC 
LIMIT 10;"

# Check application metrics
curl https://your-domain.com/api/metrics
```

**Solutions:**
1. **Database optimization:**
   ```bash
   # Add missing indexes
   docker-compose exec backend psql $DATABASE_URL -c "
   CREATE INDEX CONCURRENTLY idx_messages_session_id ON messages(session_id);
   CREATE INDEX CONCURRENTLY idx_documents_created_at ON documents(created_at);"
   ```

2. **Scale horizontally:**
   ```bash
   # Docker Compose
   docker-compose -f docker-compose.prod.yml up -d --scale backend=3
   
   # Kubernetes
   kubectl scale deployment rag-backend --replicas=5 -n rag-chatbot
   ```

3. **Enable caching:**
   ```bash
   # Update environment variables
   echo "ENABLE_RESPONSE_CACHE=true" >> .env.prod
   docker-compose -f docker-compose.prod.yml up -d
   ```

#### Issue: Service Unavailable (503 Errors)

**Symptoms:**
- 503 Service Unavailable errors
- Load balancer health checks failing
- Services not responding

**Investigation:**
```bash
# Check service status
docker-compose ps
kubectl get pods -n rag-chatbot

# Check logs
docker-compose logs backend
kubectl logs deployment/rag-backend -n rag-chatbot

# Check network connectivity
curl -v http://backend:8000/health
```

**Solutions:**
1. **Restart services:**
   ```bash
   # Docker Compose
   docker-compose restart backend
   
   # Kubernetes
   kubectl rollout restart deployment/rag-backend -n rag-chatbot
   ```

2. **Check resource limits:**
   ```bash
   # Increase memory limits
   # Edit docker-compose.prod.yml or k8s manifests
   memory: "4Gi"  # Increase from 2Gi
   ```

3. **Check dependencies:**
   ```bash
   # Test database connection
   docker-compose exec backend python -c "
   from book_backend.src.database.database import engine
   print(engine.execute('SELECT 1').fetchone())
   "
   
   # Test Qdrant connection
   curl -f $QDRANT_URL/collections
   ```

#### Issue: Memory Leaks

**Symptoms:**
- Gradually increasing memory usage
- Out of memory errors
- Container restarts

**Investigation:**
```bash
# Monitor memory usage over time
docker stats --no-stream
kubectl top pods -n rag-chatbot

# Check for memory leaks in application
docker-compose exec backend python -c "
import psutil
import gc
print(f'Memory usage: {psutil.virtual_memory().percent}%')
print(f'Objects in memory: {len(gc.get_objects())}')
"
```

**Solutions:**
1. **Restart services periodically:**
   ```bash
   # Add to crontab
   0 2 * * * docker-compose restart backend
   ```

2. **Optimize embedding service:**
   ```bash
   # Clear model cache
   docker-compose exec backend python -c "
   import torch
   torch.cuda.empty_cache()
   "
   ```

3. **Implement connection pooling:**
   ```python
   # Update database configuration
   SQLALCHEMY_POOL_SIZE = 10
   SQLALCHEMY_MAX_OVERFLOW = 20
   SQLALCHEMY_POOL_RECYCLE = 3600
   ```

#### Issue: Authentication Failures

**Symptoms:**
- 401 Unauthorized errors
- Users unable to login
- Session timeouts

**Investigation:**
```bash
# Check JWT configuration
echo $JWT_SECRET_KEY | wc -c  # Should be > 32 characters

# Check session storage
docker-compose exec redis redis-cli KEYS "session:*"

# Check authentication logs
grep "auth" /var/log/rag-chatbot/app.log
```

**Solutions:**
1. **Regenerate JWT secret:**
   ```bash
   # Generate new secret
   openssl rand -hex 32
   
   # Update environment
   echo "JWT_SECRET_KEY=new_secret" >> .env.prod
   docker-compose -f docker-compose.prod.yml up -d
   ```

2. **Clear invalid sessions:**
   ```bash
   docker-compose exec redis redis-cli FLUSHDB
   ```

### Performance Optimization

#### Database Optimization
```sql
-- Add indexes for common queries
CREATE INDEX CONCURRENTLY idx_messages_session_created 
ON messages(session_id, created_at);

CREATE INDEX CONCURRENTLY idx_documents_status 
ON documents(status) WHERE status = 'completed';

-- Analyze query performance
EXPLAIN ANALYZE SELECT * FROM messages 
WHERE session_id = 'uuid' 
ORDER BY created_at DESC LIMIT 10;

-- Update table statistics
ANALYZE messages;
ANALYZE documents;
ANALYZE sessions;
```

#### Application Optimization
```bash
# Enable response compression
echo "ENABLE_GZIP=true" >> .env.prod

# Optimize embedding batch size
echo "EMBEDDING_BATCH_SIZE=32" >> .env.prod

# Enable connection pooling
echo "DB_POOL_SIZE=20" >> .env.prod
echo "DB_MAX_OVERFLOW=30" >> .env.prod
```

#### Caching Strategy
```bash
# Enable Redis caching
echo "REDIS_URL=redis://redis:6379/0" >> .env.prod
echo "CACHE_TTL=3600" >> .env.prod

# Cache frequently accessed data
curl -X POST https://your-domain.com/api/admin/cache/warm
```

## 📊 Monitoring and Alerting

### Key Alerts to Configure

#### Critical Alerts (P0)
- Service down (health check fails)
- Error rate > 5%
- Response time > 10 seconds
- Memory usage > 95%
- Disk usage > 95%

#### Warning Alerts (P1)
- Error rate > 1%
- Response time > 5 seconds
- Memory usage > 85%
- CPU usage > 80%
- Database connection pool > 80%

#### Info Alerts (P2)
- Deployment completed
- Backup completed
- Certificate expiring (30 days)
- High traffic volume

### Monitoring Queries

#### Prometheus Queries
```promql
# Error rate
rate(http_requests_total{status=~"5.."}[5m]) / rate(http_requests_total[5m])

# Response time 95th percentile
histogram_quantile(0.95, rate(http_request_duration_seconds_bucket[5m]))

# Memory usage
(node_memory_MemTotal_bytes - node_memory_MemAvailable_bytes) / node_memory_MemTotal_bytes

# CPU usage
100 - (avg by (instance) (rate(node_cpu_seconds_total{mode="idle"}[5m])) * 100)
```

#### Log-based Alerts
```bash
# Error spike detection
tail -f /var/log/rag-chatbot/app.log | grep -i error | wc -l

# Performance degradation
grep "response_time" /var/log/rag-chatbot/app.log | awk '$5 > 5000'

# Authentication failures
grep "401\|403" /var/log/rag-chatbot/access.log | wc -l
```

## 🔄 Maintenance Procedures

### Daily Maintenance
```bash
#!/bin/bash
# daily-maintenance.sh

# Check system health
curl -f https://your-domain.com/api/health || echo "Health check failed"

# Check disk space
df -h | awk '$5 > 80 {print "Disk usage warning: " $0}'

# Check log file sizes
find /var/log/rag-chatbot -name "*.log" -size +100M

# Backup database
./scripts/backup-db.sh

# Clean old logs
find /var/log/rag-chatbot -name "*.log.*" -mtime +7 -delete

# Update system packages (if needed)
apt list --upgradable | grep -i security
```

### Weekly Maintenance
```bash
#!/bin/bash
# weekly-maintenance.sh

# Security updates
apt update && apt upgrade -y

# Docker cleanup
docker system prune -f
docker volume prune -f

# Database maintenance
docker-compose exec backend psql $DATABASE_URL -c "VACUUM ANALYZE;"

# Certificate check
openssl x509 -in /etc/nginx/ssl/cert.pem -noout -dates

# Performance review
./scripts/performance-report.sh
```

### Monthly Maintenance
```bash
#!/bin/bash
# monthly-maintenance.sh

# Full system backup
./scripts/full-backup.sh

# Dependency updates
cd book-backend && uv sync --upgrade
cd book-frontend && npm audit fix

# Security scan
docker run --rm -v /var/run/docker.sock:/var/run/docker.sock \
  aquasec/trivy image rag-backend:latest

# Capacity planning review
./scripts/capacity-report.sh

# Disaster recovery test
./scripts/dr-test.sh
```

## 📋 Runbook Checklists

### Incident Response Checklist
- [ ] Assess severity level (P0-P3)
- [ ] Create incident channel
- [ ] Notify stakeholders
- [ ] Begin investigation
- [ ] Implement mitigation
- [ ] Monitor for resolution
- [ ] Conduct post-mortem
- [ ] Update documentation

### Deployment Checklist
- [ ] Code review completed
- [ ] Tests passing
- [ ] Backup created
- [ ] Deployment executed
- [ ] Health checks passing
- [ ] Smoke tests completed
- [ ] Monitoring verified
- [ ] Rollback plan ready

### Security Incident Checklist
- [ ] Isolate affected systems
- [ ] Preserve evidence
- [ ] Notify security team
- [ ] Assess impact
- [ ] Implement containment
- [ ] Eradicate threat
- [ ] Recover systems
- [ ] Document lessons learned

## 📞 Escalation Procedures

### When to Escalate
- **Immediate (P0)**: System down, data loss, security breach
- **1 Hour (P1)**: Major feature broken, significant impact
- **4 Hours (P2)**: Minor issues, workarounds available
- **Next Day (P3)**: Enhancement requests, documentation

### Escalation Contacts
```
L1 Support: support@company.com
L2 Engineering: engineering@company.com
L3 Management: management@company.com
L4 Executive: executive@company.com

External Vendors:
- Neon Support: support@neon.tech
- Qdrant Support: support@qdrant.io
- Google Cloud: cloud-support@google.com
```

### Communication Templates

#### Incident Notification
```
Subject: [P0] RAG Chatbot System Down

Impact: Complete service unavailability
Start Time: 2024-01-01 12:00 UTC
Affected Users: All users
Current Status: Investigating

Actions Taken:
- Incident response team activated
- Initial investigation started
- Monitoring systems checked

Next Update: 15 minutes

Incident Commander: John Doe
```

#### Resolution Notification
```
Subject: [RESOLVED] RAG Chatbot System Restored

The incident has been resolved.

Resolution Time: 2024-01-01 12:45 UTC
Duration: 45 minutes
Root Cause: Database connection pool exhaustion

Actions Taken:
- Increased connection pool size
- Restarted affected services
- Verified system stability

Post-Mortem: Scheduled for 2024-01-02 10:00 UTC
```

---

## 📚 Additional Resources

- [Deployment Guide](./deployment.md)
- [Monitoring Setup](./monitoring.md)
- [Security Best Practices](./security.md)
- [API Documentation](../book-backend/docs/api_documentation.md)

For questions or updates to this runbook, contact the DevOps team or create an issue in the repository.