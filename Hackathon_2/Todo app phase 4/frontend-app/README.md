# Task Management Frontend

A modern Next.js 16 frontend application for task management with AI-powered chat assistance.

## 🐳 Docker Container

### Container Details

- **Base Image**: `node:20-alpine` (builder), `nginx:alpine` (runtime)
- **Image Size**: ~95MB
- **Port**: 80
- **User**: nginx (non-root)

### Building the Container

```bash
cd frontend-app
docker build -t todo-frontend:latest .
```

**Build Time**: ~3-5 minutes (first build), ~1-2 minutes (cached builds)

### Running the Container

**Standalone Mode:**
```bash
docker run -d \
  --name todo-frontend \
  -p 80:80 \
  todo-frontend:latest
```

Access at `http://localhost`

**With Backend Service:**
```bash
# Create network
docker network create todo-network

# Run backend
docker run -d \
  --name todo-backend \
  --network todo-network \
  -p 8000:8000 \
  -e DATABASE_URL="..." \
  -e SECRET_KEY="..." \
  todo-backend:latest

# Run frontend
docker run -d \
  --name todo-frontend \
  --network todo-network \
  -p 80:80 \
  todo-frontend:latest
```

### Container Health Check

Test health endpoint:
```bash
curl http://localhost/health
# Expected: healthy
```

### View Container Logs

```bash
# View all logs
docker logs todo-frontend

# Follow logs in real-time
docker logs -f todo-frontend
```

For detailed container documentation, see the [Containerization Guide](../specs/009-containerization/quickstart.md).

## Getting Started

First, run the development server:

```bash
npm run dev
# or
yarn dev
# or
pnpm dev
# or
bun dev
```

Open [http://localhost:3000](http://localhost:3000) with your browser to see the result.

You can start editing the page by modifying `app/page.tsx`. The page auto-updates as you edit the file.

This project uses [`next/font`](https://nextjs.org/docs/app/building-your-application/optimizing/fonts) to automatically optimize and load [Geist](https://vercel.com/font), a new font family for Vercel.

## Learn More

To learn more about Next.js, take a look at the following resources:

- [Next.js Documentation](https://nextjs.org/docs) - learn about Next.js features and API.
- [Learn Next.js](https://nextjs.org/learn) - an interactive Next.js tutorial.

You can check out [the Next.js GitHub repository](https://github.com/vercel/next.js) - your feedback and contributions are welcome!

## Deploy on Vercel

The easiest way to deploy your Next.js app is to use the [Vercel Platform](https://vercel.com/new?utm_medium=default-template&filter=next.js&utm_source=create-next-app&utm_campaign=create-next-app-readme) from the creators of Next.js.

Check out our [Next.js deployment documentation](https://nextjs.org/docs/app/building-your-application/deploying) for more details.
