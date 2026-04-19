#!/bin/bash
# scripts/deploy-to-cloud.sh
# Deploy Platinum AI Employee to Cloud VM (Oracle Cloud or AWS)

set -e

echo "=== Platinum AI Employee Cloud Deployment ==="
echo ""

# Select cloud provider
echo "Select cloud provider:"
echo "1) Oracle Cloud Free Tier (Recommended - 4 OCPU, 24GB RAM)"
echo "2) AWS EC2 (t3.medium - 2 vCPU, 8GB RAM)"
echo "3) Manual SSH Setup"
read -p "Enter choice (1-3): " provider_choice

case $provider_choice in
    1)
        echo ""
        echo "=== Oracle Cloud Deployment ==="
        read -p "Enter Compartment ID: " COMPARTMENT_ID
        read -p "Enter Availability Domain: " AVAILABILITY_DOMAIN
        read -p "Enter Subnet ID: " SUBNET_ID
        read -p "Enter SSH Public Key Path (~/.ssh/id_rsa.pub): " SSH_KEY_PATH
        
        SSH_KEY_PATH=${SSH_KEY_PATH/#\~/$HOME}
        
        echo "Creating Oracle Cloud VM instance..."
        
        VM_OCID=$(oci compute instance launch \
            --compartment-id "$COMPARTMENT_ID" \
            --shape "VM.Standard.A1.Flex" \
            --shape-memory 24 \
            --image-id "ocid1.image.oc1..aaaaaaaavvu3qf3l5q5q5q5q5q5q5q5q5q5q5q5q5q5q5q5q5q5q5q5q" \
            --availability-domain "$AVAILABILITY_DOMAIN" \
            --subnet-id "$SUBNET_ID" \
            --assign-public-ip true \
            --ssh-authorized-keys-file "$SSH_KEY_PATH" \
            --display-name "ai-employee-cloud" \
            --metadata-file <(cat <<EOF
{
    "user_data": "$(base64 -w0 cloud/cloud-init.yaml)"
}
EOF
) \
            --query 'data.id' \
            --output text)
        
        echo "VM creation initiated..."
        echo "VM OCID: $VM_OCID"
        echo "Waiting for VM to be provisioned (this takes 3-5 minutes)..."
        
        # Wait for VM to be running
        while true; do
            STATUS=$(oci compute instance get --instance-id "$VM_OCID" --query 'data."lifecycle-state"' --output text)
            echo "  Current status: $STATUS"
            if [ "$STATUS" = "RUNNING" ]; then
                break
            fi
            sleep 30
        done
        
        # Get public IP
        PUBLIC_IP=$(oci compute instance get --instance-id "$VM_OCID" \
            --query 'data."public-ip"' \
            --output text)
        
        echo ""
        echo "✓ VM provisioned successfully!"
        echo "Public IP: $PUBLIC_IP"
        ;;
        
    2)
        echo ""
        echo "=== AWS EC2 Deployment ==="
        read -p "Enter AMI ID (Amazon Linux 2): " AMI_ID
        read -p "Enter Key Pair Name: " KEY_NAME
        read -p "Enter Security Group ID: " SECURITY_GROUP_ID
        read -p "Enter Subnet ID: " SUBNET_ID
        
        echo "Creating EC2 instance..."
        
        INSTANCE_ID=$(aws ec2 run-instances \
            --image-id "$AMI_ID" \
            --instance-type t3.medium \
            --key-name "$KEY_NAME" \
            --security-group-ids "$SECURITY_GROUP_ID" \
            --subnet-id "$SUBNET_ID" \
            --block-device-mappings '[{"DeviceName":"/dev/xvda","Ebs":{"VolumeSize":50}}]' \
            --tag-specifications '[{"ResourceType":"instance","Tags":[{"Key":"Name","Value":"ai-employee-cloud"}]}]' \
            --query 'Instances[0].InstanceId' \
            --output text)
        
        echo "Instance created: $INSTANCE_ID"
        echo "Waiting for instance to be running..."
        
        aws ec2 wait instance-running --instance-ids "$INSTANCE_ID"
        
        # Get public IP
        PUBLIC_IP=$(aws ec2 describe-instances \
            --instance-ids "$INSTANCE_ID" \
            --query 'Reservations[0].Instances[0].PublicIpAddress' \
            --output text)
        
        echo ""
        echo "✓ EC2 instance running!"
        echo "Public IP: $PUBLIC_IP"
        ;;
        
    3)
        echo ""
        read -p "Enter Cloud VM IP address: " PUBLIC_IP
        read -p "Enter SSH username (default: ubuntu): " SSH_USER
        SSH_USER=${SSH_USER:-ubuntu}
        ;;
        
    *)
        echo "Invalid choice"
        exit 1
        ;;
esac

echo ""
echo "=== Setting up Cloud VM ==="

if [ -z "$PUBLIC_IP" ]; then
    echo "Error: No public IP obtained"
    exit 1
fi

# SSH into VM and setup
echo "Connecting to $PUBLIC_IP..."

# Copy deployment script to VM
scp -i "$SSH_KEY_PATH" cloud/setup-cloud-vm.sh ${SSH_USER:-ubuntu}@$PUBLIC_IP:~/setup-cloud-vm.sh

# Execute setup script
ssh -i "$SSH_KEY_PATH" ${SSH_USER:-ubuntu}@$PUBLIC_IP << 'EOF'
chmod +x setup-cloud-vm.sh
./setup-cloud-vm.sh
EOF

echo ""
echo "=== Cloud VM Setup Complete ==="
echo ""
echo "Next steps:"
echo "1. Copy your .env file to Cloud VM:"
echo "   scp .env ${SSH_USER:-ubuntu}@$PUBLIC_IP:/opt/ai-employee/.env"
echo ""
echo "2. Initialize vault sync:"
echo "   ./scripts/setup-vault-sync.sh"
echo ""
echo "3. Check Cloud health:"
echo "   curl https://$PUBLIC_IP/health"
echo ""
echo "4. Start Local agent:"
echo "   python local/orchestrator.py"
echo ""
