# 🚀 Setup: Temporary Server for Instagram (NO Third-Party Hosting!)

## Overview

This solution creates a **temporary server on your computer** with a **public URL tunnel** using ngrok. 

**Workflow:**
1. Start local HTTP server (your computer)
2. Create ngrok tunnel (temporary public URL)
3. Instagram fetches image from that URL
4. Close server and tunnel
5. **Result:** Image on Instagram, NO permanent hosting!

---

## **Step 1: Get ngrok Auth Token (3 minutes)**

### **Sign Up for ngrok (FREE)**

1. **Go to:** https://dashboard.ngrok.com/signup

2. **Sign up** using:
   - Google account, OR
   - GitHub account, OR
   - Email + password

3. **Verify your email** (if using email signup)

---

### **Get Your Auth Token**

1. **Login** to: https://dashboard.ngrok.com/get-started/your-authtoken

2. **Copy your auth token** (looks like: `2AbCdEfGhIjKlMnOpQrStUvWxYz123456789`)

---

## **Step 2: Add ngrok Token to .env**

Open: `E:\hackathon-0\Gold\.env`

Find line 36:
```bash
NGROK_AUTH_TOKEN=your_ngrok_auth_token_here
```

Replace with your actual token:
```bash
NGROK_AUTH_TOKEN=2AbCdEfGhIjKlMnOpQrStUvWxYz123456789
```

Save the file.

---

## **Step 3: Copy MCP Config to Qwen Code**

The config is already updated at: `E:\hackathon-0\Gold\mcp-config.json`

**Copy the entire file** to:
- **Windows:** `%APPDATA%\qwen\mcp.json`
  - Or: `C:\Users\FAIZ REHMAN\AppData\Roaming\qwen\mcp.json`

**OR** just add this to your existing mcp.json:

```json
"instagram-temporary": {
  "command": "python",
  "args": ["E:/hackathon-0/Gold/AI_Employee_Vault/mcp_servers/social_media/instagram_temporary_server.py"],
  "env": {
    "NGROK_AUTH_TOKEN": "YOUR_TOKEN_HERE",
    "INSTAGRAM_ACCESS_TOKEN": "EAAY...",
    "INSTAGRAM_ACCOUNT_ID": "17841446919185197",
    "DRY_RUN": "false"
  }
}
```

---

## **Step 4: Restart Qwen Code**

Close and reopen Qwen Code to load the new MCP server.

---

## **Step 5: Test It! 🧪**

Run this command in Qwen Code:

```bash
/social-poster --platform instagram "Ramadan Mubarak! 🌙✨" --media-path "E:/hackathon-0/Gold/ramadan_mubarak.jpg"
```

**Expected Result:**
```
✅ Local server started on port 8080
✅ ngrok tunnel created: https://abc123.ngrok.io/ramadan_mubarak.jpg
✅ Media container created: 18042302951563973
✅ Instagram post successful
✅ Temporary server and tunnel closed
✅ Posted to Instagram ONLY (no permanent hosting!)
```

---

## **How It Works**

```
┌─────────────────────────────────────────────────────────────┐
│ 1. User requests Instagram post with local file             │
│    /social-poster --media-path "C:/path/to/image.jpg"       │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 2. Start local HTTP server on your computer                 │
│    Port: 8080                                                │
│    Serves only the specific image file                       │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 3. Create ngrok tunnel (temporary public URL)               │
│    https://abc123.ngrok.io/image.jpg                         │
│    This URL is public but TEMPORARY                          │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 4. Use ngrok URL to create Instagram post                   │
│    POST /{account_id}/media                                  │
│    image_url: https://abc123.ngrok.io/image.jpg              │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 5. Publish to Instagram                                     │
│    POST /{account_id}/media_publish                          │
│    Returns: post_id, URL                                     │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 6. CLOSE server and ngrok tunnel                            │
│    Local server: SHUTDOWN                                    │
│    ngrok tunnel: DISCONNECTED                                │
│    Public URL: NO LONGER WORKS                               │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 7. Result: Image on Instagram ONLY!                         │
│    ❌ No Facebook post                                       │
│    ❌ No permanent hosting                                   │
│    ❌ No third-party image storage                           │
│    ✅ You control everything                                 │
└─────────────────────────────────────────────────────────────┘
```

---

## **Comparison: All Methods**

| Method | Third-Party | Facebook Post | Permanent Hosting | You Control |
|--------|-------------|---------------|-------------------|-------------|
| **Temporary Server (NEW)** | ❌ No* | ❌ No | ❌ No | ✅ Yes |
| Imgur | ✅ Yes | ❌ No | ✅ Yes | ⚠️ Partial |
| Facebook Workaround | ✅ Yes | ✅ Yes | ✅ Yes | ⚠️ Partial |
| Your Own Server | ❌ No | ❌ No | ✅ Yes | ✅ Yes |

*ngrok is a tunnel, not hosting - it just forwards traffic to your computer

---

## **ngrok Free Tier Limits**

| Limit | Value |
|-------|-------|
| **Cost** | Free |
| **Tunnels** | 1 at a time |
| **Bandwidth** | Unlimited |
| **URL Type** | Random (changes each time) |
| **Duration** | As long as tunnel is active |
| **Privacy** | Tunnel closes when done |

---

## **Troubleshooting**

### **"pyngrok not installed"**
```bash
pip install pyngrok
```

### **"ngrok auth token not configured"**
- Check `NGROK_AUTH_TOKEN` in `.env`
- Make sure you copied your token from ngrok dashboard

### **"Port 8080 already in use"**
- Close any other programs using port 8080
- Or change port in `instagram_temporary_server.py`:
  ```python
  'server_port': 8081,  # Change to different port
  ```

### **"ngrok tunnel failed"**
- Check your internet connection
- Verify ngrok token is correct
- Try logging out and back into ngrok dashboard

### **"Instagram credentials not configured"**
- Check `INSTAGRAM_ACCESS_TOKEN` and `INSTAGRAM_ACCOUNT_ID` in `.env`

---

## **Privacy & Security**

### **Is This Safe?**

✅ **Yes!** Here's why:

1. **Temporary**: Tunnel only exists during upload (seconds)
2. **Specific**: Only serves the one image file
3. **Controlled**: You start and stop it
4. **No Storage**: Image not stored anywhere permanent
5. **Firewall**: ngrok handles the public exposure, not your firewall

### **What ngrok Sees**

- ngrok sees the image traffic (encrypted via HTTPS)
- ngrok does NOT store your images
- ngrok is just a tunnel (like a pipe)
- Free tier has random URLs (can't predict/reuse)

---

## **Summary**

| Aspect | Details |
|--------|---------|
| **Best For** | Instagram-only posting, no third-party hosting |
| **Hosting** | Your computer (temporary) |
| **Facebook Posts** | ❌ None created |
| **Instagram Posts** | ✅ Yes |
| **Setup Time** | 5 minutes (includes ngrok signup) |
| **Cost** | Free (ngrok free tier) |
| **Privacy** | High (temporary, no storage) |
| **Recommended** | ✅ YES! Best option |

---

## **Next Steps**

1. ✅ **Get ngrok auth token** (3 min)
2. ✅ **Add to `.env`**
3. ✅ **Copy mcp-config.json to Qwen Code**
4. ✅ **Restart Qwen Code**
5. ✅ **Test with ramadan_mubarak.jpg**
6. 🎉 **Enjoy private Instagram posting!**

---

**Questions? Come back and I'll help you test!** 🚀

---

**References:**
- [ngrok Documentation](https://ngrok.com/docs)
- [Instagram Graph API](https://developers.facebook.com/docs/instagram-api)
- [pyngrok Python Package](https://github.com/alanhamlett/pyngrok)
