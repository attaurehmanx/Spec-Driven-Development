#!/bin/bash
# cloud/setup-cloud-vm.sh
# Setup script for Cloud VM (run after initial deployment)

set -e

echo "=== Setting up Platinum AI Employee Cloud VM ==="

# Configuration
AI_EMPLOYEE_DIR="/opt/ai-employee"
GIT_REPO_URL="${GIT_REPO_URL:-}"

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root"
    exit 1
fi

# Install dependencies
echo "Installing dependencies..."
apt-get update
apt-get install -y \
    docker.io \
    docker-compose \
    python3 \
    python3-pip \
    python3-venv \
    git \
    nginx \
    certbot \
    python3-certbot-nginx \
    curl \
    wget \
    htop \
    jq \
    mailutils

# Create ai-employee user if not exists
if ! id -u ai-employee > /dev/null 2>&1; then
    echo "Creating ai-employee user..."
    adduser --disabled-password --gecos "" ai-employee
    usermod -aG sudo ai-employee
    usermod -aG docker ai-employee
    echo "ai-employee ALL=(ALL) NOPASSWD:ALL" | tee /etc/sudoers.d/ai-employee
fi

# Create directory structure
echo "Creating directory structure..."
mkdir -p $AI_EMPLOYEE_DIR/{vault,logs,odoo/config,odoo/addons,odoo/backups,ssl,scripts,docker}
mkdir -p $AI_EMPLOYEE_DIR/vault/{Inbox,Needs_Action/cloud,Needs_Action/local,In_Progress/cloud_agent,In_Progress/local_agent,Pending_Approval,Approved,Done/cloud,Done/local,Updates,Signals,Logs}
chown -R ai-employee:ai-employee $AI_EMPLOYEE_DIR
chmod -R 755 $AI_EMPLOYEE_DIR

# Setup Docker Compose
echo "Setting up Docker Compose..."
cat > $AI_EMPLOYEE_DIR/docker-compose.yml << 'DOCKERCOMPOSE'
version: '3.8'

services:
  cloud-agent:
    build: ./docker/cloud-agent
    container_name: cloud-agent
    restart: always
    volumes:
      - ./vault:/app/vault
      - ./logs:/app/logs
      - ./cloud:/app/cloud
    environment:
      - ENVIRONMENT=cloud
      - AGENT_ROLE=drafts_only
    networks:
      - ai-network
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8080/health"]
      interval: 60s
      timeout: 10s
      retries: 3

  odoo:
    image: odoo:17.0
    container_name: odoo
    restart: always
    ports:
      - "127.0.0.1:8069:8069"
    environment:
      - HOST=db
      - USER=odoo
      - PASSWORD=${ODOO_DB_PASSWORD:-odoo}
    volumes:
      - odoo-data:/var/lib/odoo
      - ./odoo/config:/etc/odoo
      - ./odoo/addons:/mnt/extra-addons
    depends_on:
      - db
    networks:
      - ai-network

  db:
    image: postgres:15
    container_name: odoo-db
    restart: always
    environment:
      - POSTGRES_DB=odoo
      - POSTGRES_USER=odoo
      - POSTGRES_PASSWORD=${ODOO_DB_PASSWORD:-odoo}
    volumes:
      - postgres-data:/var/lib/postgresql/data
      - ./odoo/backups:/backups
    networks:
      - ai-network

  vault-sync:
    build: ./docker/vault-sync
    container_name: vault-sync
    restart: always
    volumes:
      - ./vault:/app/vault
      - ./sync-logs:/app/logs
    environment:
      - GIT_REPO_URL=${GIT_REPO_URL}
      - GIT_EMAIL=${GIT_EMAIL:-ai-employee@local}
      - SYNC_INTERVAL=300
    depends_on:
      - cloud-agent
    networks:
      - ai-network

volumes:
  odoo-data:
  postgres-data:

networks:
  ai-network:
    driver: bridge
DOCKERCOMPOSE

# Setup Nginx configuration
echo "Setting up Nginx..."
cat > $AI_EMPLOYEE_DIR/nginx.conf << 'NGINXCONF'
events {
    worker_connections 1024;
}

http {
    server {
        listen 80;
        server_name _;
        
        location /.well-known/acme-challenge/ {
            root /var/www/certbot;
        }
        
        location /health {
            alias /usr/share/nginx/html/health.json;
            default_type application/json;
        }
        
        location / {
            return 301 https://$server_name$request_uri;
        }
    }

    server {
        listen 443 ssl http2;
        server_name _;

        ssl_certificate /etc/nginx/ssl/fullchain.pem;
        ssl_certificate_key /etc/nginx/ssl/privkey.pem;

        ssl_protocols TLSv1.2 TLSv1.3;
        ssl_ciphers HIGH:!aNULL:!MD5;
        ssl_prefer_server_ciphers on;

        location /health {
            alias /usr/share/nginx/html/health.json;
            default_type application/json;
        }

        location / {
            return 404;
        }
    }
}
NGINXCONF

# Setup health check endpoint
echo "Setting up health endpoint..."
cat > $AI_EMPLOYEE_DIR/health.json << 'HEALTHJSON'
{
    "status": "healthy",
    "service": "platinum-cloud-agent"
}
HEALTHJSON

# Create backup script
echo "Creating backup script..."
cat > $AI_EMPLOYEE_DIR/scripts/backup-odoo.sh << 'BACKUPSCRIPT'
#!/bin/bash
set -e

BACKUP_DIR="/opt/ai-employee/odoo/backups"
DATE=$(date +%Y%m%d_%H%M%S)

echo "=== Odoo Backup ==="
mkdir -p $BACKUP_DIR

# Backup database
docker exec odoo-db pg_dump -U odoo odoo > $BACKUP_DIR/db_$DATE.sql
gzip $BACKUP_DIR/db_$DATE.sql

# Backup filestore
docker run --rm \
    -v odoo-odoo-data:/data \
    -v $BACKUP_DIR:/backup \
    alpine tar czf /backup/filestore_$DATE.tar.gz -C /data .

# Cleanup old backups (keep 30 days)
find $BACKUP_DIR -name "*.sql.gz" -mtime +30 -delete
find $BACKUP_DIR -name "*.tar.gz" -mtime +30 -delete

echo "Backup complete: $BACKUP_DIR"
BACKUPSCRIPT
chmod +x $AI_EMPLOYEE_DIR/scripts/backup-odoo.sh

# Create health monitoring script
echo "Creating health monitoring script..."
cat > $AI_EMPLOYEE_DIR/scripts/health-monitor.sh << 'HEALTHSCRIPT'
#!/bin/bash

CLOUD_VM_IP=$(curl -s http://169.254.169.254/latest/meta-data/public-ipv4 2>/dev/null || echo "localhost")
ALERT_EMAIL="${ALERT_EMAIL:-}"

echo "=== Health Monitor ==="

# Check health endpoint
HEALTH_RESPONSE=$(curl -s http://$CLOUD_VM_IP/health 2>/dev/null || echo '{"status":"error"}')
STATUS=$(echo $HEALTH_RESPONSE | jq -r '.status // "error"')

echo "Status: $STATUS"

if [ "$STATUS" = "error" ]; then
    echo "WARNING: Health check failed!"
    if [ -n "$ALERT_EMAIL" ]; then
        echo "Health check failed" | mail -s "AI Employee Alert" $ALERT_EMAIL
    fi
fi

# Check container status
echo "Container Status:"
docker ps --format "table {{.Names}}\t{{.Status}}"

# Check disk usage
echo "Disk Usage:"
df -h / | tail -1

# Check memory
echo "Memory Usage:"
free -h | grep Mem
HEALTHSCRIPT
chmod +x $AI_EMPLOYEE_DIR/scripts/health-monitor.sh

# Setup systemd service for cloud agent
echo "Setting up systemd service..."
cat > /etc/systemd/system/cloud-agent.service << 'SYSTEMDSERVICE'
[Unit]
Description=Platinum AI Employee Cloud Agent
After=docker.service
Requires=docker.service

[Service]
Type=oneshot
RemainAfterExit=yes
WorkingDirectory=/opt/ai-employee
ExecStart=/usr/bin/docker-compose up -d
ExecStop=/usr/bin/docker-compose down
TimeoutStartSec=0

[Install]
WantedBy=multi-user.target
SYSTEMDSERVICE

# Enable services
echo "Enabling services..."
systemctl daemon-reload
systemctl enable cloud-agent

# Setup log rotation
echo "Setting up log rotation..."
cat > /etc/logrotate.d/ai-employee << 'LOGROTATE'
/opt/ai-employee/logs/*.log {
    daily
    rotate 30
    compress
    delaycompress
    notifempty
    create 0644 ai-employee ai-employee
}
LOGROTATE

# Setup cron jobs
echo "Setting up cron jobs..."
(crontab -l 2>/dev/null; echo "0 2 * * * /opt/ai-employee/scripts/backup-odoo.sh") | crontab -u ai-employee -
(crontab -l 2>/dev/null; echo "*/5 * * * * /opt/ai-employee/scripts/health-monitor.sh >> /var/log/ai-employee-health.log") | crontab -u ai-employee -

# Get public IP
PUBLIC_IP=$(curl -s http://169.254.169.254/latest/meta-data/public-ipv4 2>/dev/null || hostname -I | awk '{print $1}')

echo ""
echo "=== Cloud VM Setup Complete ==="
echo ""
echo "Public IP: $PUBLIC_IP"
echo "Health Endpoint: http://$PUBLIC_IP/health"
echo ""
echo "Next steps:"
echo "1. Copy .env file: scp .env ai-employee@$PUBLIC_IP:/opt/ai-employee/.env"
echo "2. Clone repository: ssh ai-employee@$PUBLIC_IP 'cd /opt/ai-employee && git clone <repo-url> .'"
echo "3. Start services: ssh ai-employee@$PUBLIC_IP 'cd /opt/ai-employee && sudo systemctl start cloud-agent'"
echo "4. Check health: curl http://$PUBLIC_IP/health"
echo ""
