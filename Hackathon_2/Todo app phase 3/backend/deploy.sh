#!/bin/bash
# Quick deployment script for Hugging Face Spaces

echo "🚀 Hugging Face Spaces Deployment Helper"
echo "=========================================="
echo ""

# Check if we're in the backend directory
if [ ! -f "main.py" ]; then
    echo "❌ Error: Please run this script from the backend directory"
    exit 1
fi

echo "📋 Pre-deployment Checklist:"
echo ""
echo "1. Environment Variables (set in HF Spaces settings):"
echo "   ✓ DATABASE_URL"
echo "   ✓ JWT_SECRET_KEY"
echo "   ✓ BETTER_AUTH_SECRET"
echo "   ✓ GEMINI_API_KEY"
echo ""

echo "2. Files to deploy:"
echo "   ✓ main.py, app.py, run_server.py"
echo "   ✓ requirements.txt"
echo "   ✓ Dockerfile, .dockerignore"
echo "   ✓ api/, models/, services/, database/, middleware/, utils/, config/"
echo "   ✓ uploads/ (with default avatars)"
echo ""

echo "3. Files excluded (via .dockerignore):"
echo "   ✓ tests/ (test files)"
echo "   ✓ docs/ (documentation)"
echo "   ✓ scripts/ (utility scripts)"
echo "   ✓ venv/ (virtual environment)"
echo "   ✓ .env (secrets)"
echo ""

echo "📦 Deployment Options:"
echo ""
echo "Option A: Git Push"
echo "  git clone https://huggingface.co/spaces/YOUR_USERNAME/your-space"
echo "  cd your-space"
echo "  cp -r /path/to/backend/* ."
echo "  git add ."
echo "  git commit -m 'Deploy backend'"
echo "  git push"
echo ""

echo "Option B: Hugging Face CLI"
echo "  huggingface-cli login"
echo "  huggingface-cli upload YOUR_USERNAME/your-space . --repo-type=space"
echo ""

echo "🔍 Post-deployment Verification:"
echo ""
echo "1. Health check:"
echo "   curl https://YOUR_SPACE.hf.space/health"
echo ""
echo "2. API docs:"
echo "   https://YOUR_SPACE.hf.space/docs"
echo ""
echo "3. Test signup:"
echo "   curl -X POST https://YOUR_SPACE.hf.space/auth/signup \\"
echo "     -H 'Content-Type: application/json' \\"
echo "     -d '{\"email\":\"test@example.com\",\"password\":\"Test123!\",\"name\":\"Test User\"}'"
echo ""

echo "✅ Ready to deploy!"
echo ""
echo "📚 For detailed instructions, see DEPLOYMENT.md"
