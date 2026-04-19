#!/bin/bash
# scripts/health-monitor.sh
# Health monitoring for Cloud VM

set -e

CLOUD_VM_IP="${CLOUD_VM_IP:-localhost}"
ALERT_EMAIL="${ALERT_EMAIL:-}"
HEALTH_URL="http://$CLOUD_VM_IP/health"

echo "=== Health Monitor ==="
echo "Cloud VM: $CLOUD_VM_IP"
echo "Timestamp: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo ""

# Check health endpoint
echo "Checking health endpoint..."
HEALTH_RESPONSE=$(curl -s --connect-timeout 10 "$HEALTH_URL" 2>/dev/null || echo '{"status":"error","message":"Connection failed"}')

STATUS=$(echo $HEALTH_RESPONSE | jq -r '.status // "error"')
UPTIME=$(echo $HEALTH_RESPONSE | jq -r '.uptime // "unknown"')
CPU=$(echo $HEALTH_RESPONSE | jq -r '.cpu_percent // "unknown"')
MEMORY=$(echo $HEALTH_RESPONSE | jq -r '.memory_percent // "unknown"')
DISK=$(echo $HEALTH_RESPONSE | jq -r '.disk_percent // "unknown"')
LAST_SYNC=$(echo $HEALTH_RESPONSE | jq -r '.last_sync // "unknown"')

echo "Status: $STATUS"
echo "Uptime: $UPTIME"
echo "CPU: $CPU%"
echo "Memory: $MEMORY%"
echo "Disk: $DISK%"
echo "Last Sync: $LAST_SYNC"
echo ""

# Check for critical issues
ALERT_NEEDED=false
ALERT_MESSAGE=""

if [ "$STATUS" = "critical" ]; then
    ALERT_NEEDED=true
    ALERT_MESSAGE="Cloud VM in critical state: CPU=$CPU%, Memory=$MEMORY%, Disk=$DISK%"
    echo "⚠ CRITICAL: $ALERT_MESSAGE"
elif [ "$STATUS" = "error" ]; then
    ALERT_NEEDED=true
    ALERT_MESSAGE="Cloud VM health check failed - connection error"
    echo "⚠ ERROR: $ALERT_MESSAGE"
fi

# Check sync lag
if [ "$LAST_SYNC" != "null" ] && [ "$LAST_SYNC" != "unknown" ]; then
    LAST_SYNC_TIME=$(date -d "$LAST_SYNC" +%s 2>/dev/null || echo "0")
    CURRENT_TIME=$(date +%s)
    SYNC_LAG=$(( (CURRENT_TIME - LAST_SYNC_TIME) / 60 ))
    
    echo "Sync Lag: ${SYNC_LAG} minutes"
    
    if [ $SYNC_LAG -gt 30 ]; then
        ALERT_NEEDED=true
        if [ -n "$ALERT_MESSAGE" ]; then
            ALERT_MESSAGE="$ALERT_MESSAGE; Sync lag=${SYNC_LAG}min"
        else
            ALERT_MESSAGE="Vault sync lag: ${SYNC_LAG} minutes"
        fi
        echo "⚠ WARNING: Sync lag > 30 minutes!"
    fi
fi

# Send alert if needed
if [ "$ALERT_NEEDED" = true ] && [ -n "$ALERT_EMAIL" ]; then
    echo "Sending alert to $ALERT_EMAIL..."
    echo "$ALERT_MESSAGE" | mail -s "AI Employee Alert - $STATUS" "$ALERT_EMAIL"
fi

# Check container status (if running locally)
if [ "$CLOUD_VM_IP" = "localhost" ]; then
    echo ""
    echo "Container Status:"
    docker ps --format "table {{.Names}}\t{{.Status}}" 2>/dev/null || echo "Docker not available"
fi

echo ""
echo "Health check complete"

# Exit with error if status is critical/error
if [ "$STATUS" = "critical" ] || [ "$STATUS" = "error" ]; then
    exit 1
fi

exit 0
