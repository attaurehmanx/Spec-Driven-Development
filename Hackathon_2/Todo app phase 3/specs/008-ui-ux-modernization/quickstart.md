# Quickstart Guide: UI/UX Modernization

**Feature**: 008-ui-ux-modernization
**Date**: 2026-02-02
**Purpose**: Developer setup and usage guide for the UI/UX modernization feature

---

## Overview

This guide helps developers set up, understand, and work with the UI/UX modernization feature, which includes dark mode, animations, and the Calm Glass design system.

---

## Prerequisites

- Node.js 18+ installed
- npm or yarn package manager
- Existing Next.js 16+ application with App Router
- Basic understanding of React and TypeScript

---

## Installation

### 1. Install Dependencies

```bash
npm install framer-motion next-themes lucide-react clsx tailwind-merge
```

**Package purposes**:
- `framer-motion`: Animation library for smooth transitions
- `next-themes`: Theme management with SSR support
- `lucide-react`: Modern icon library
- `clsx`: Conditional class name utility
- `tailwind-merge`: Tailwind class conflict resolution

### 2. Configure Tailwind CSS

Update `tailwind.config.ts`:

```typescript
import type { Config } from 'tailwindcss'

const config: Config = {
  darkMode: ['class'],
  content: [
    './src/pages/**/*.{js,ts,jsx,tsx,mdx}',
    './src/components/**/*.{js,ts,jsx,tsx,mdx}',
    './src/app/**/*.{js,ts,jsx,tsx,mdx}',
  ],
  theme: {
    extend: {
      colors: {
        background: 'hsl(var(--background))',
        foreground: 'hsl(var(--foreground))',
        card: 'hsl(var(--card))',
        'card-foreground': 'hsl(var(--card-foreground))',
        primary: 'hsl(var(--primary))',
        'primary-foreground': 'hsl(var(--primary-foreground))',
        muted: 'hsl(var(--muted))',
        'muted-foreground': 'hsl(var(--muted-foreground))',
        accent: 'hsl(var(--accent))',
        'accent-foreground': 'hsl(var(--accent-foreground))',
        border: 'hsl(var(--border))',
        input: 'hsl(var(--input))',
        ring: 'hsl(var(--ring))',
      },
      borderRadius: {
        lg: 'var(--radius)',
        md: 'calc(var(--radius) - 2px)',
        sm: 'calc(var(--radius) - 4px)',
      },
      animation: {
        'fade-in': 'fadeIn 0.3s ease-in-out',
        'slide-up': 'slideUp 0.4s ease-out',
        'bounce-dots': 'bounceDots 1.4s infinite ease-in-out',
      },
      keyframes: {
        fadeIn: {
          '0%': { opacity: '0' },
          '100%': { opacity: '1' },
        },
        slideUp: {
          '0%': { transform: 'translateY(20px)', opacity: '0' },
          '100%': { transform: 'translateY(0)', opacity: '1' },
        },
        bounceDots: {
          '0%, 80%, 100%': { transform: 'scale(0)' },
          '40%': { transform: 'scale(1)' },
        },
      },
    },
  },
  plugins: [],
}

export default config
```

### 3. Add CSS Custom Properties

Update `src/styles/globals.css`:

```css
@tailwind base;
@tailwind components;
@tailwind utilities;

@layer base {
  :root {
    --background: 210 40% 98%;
    --foreground: 222.2 84% 4.9%;
    --card: 0 0% 100%;
    --card-foreground: 222.2 84% 4.9%;
    --primary: 239 84% 67%;
    --primary-foreground: 210 40% 98%;
    --muted: 210 40% 96.1%;
    --muted-foreground: 215.4 16.3% 46.9%;
    --accent: 210 40% 96.1%;
    --accent-foreground: 222.2 47.4% 11.2%;
    --border: 214.3 31.8% 91.4%;
    --input: 214.3 31.8% 91.4%;
    --ring: 239 84% 67%;
    --radius: 0.75rem;
  }

  .dark {
    --background: 222.2 84% 4.9%;
    --foreground: 210 40% 98%;
    --card: 222.2 84% 8%;
    --card-foreground: 210 40% 98%;
    --primary: 239 84% 67%;
    --primary-foreground: 222.2 84% 4.9%;
    --muted: 217.2 32.6% 17.5%;
    --muted-foreground: 215 20.2% 65.1%;
    --accent: 217.2 32.6% 17.5%;
    --accent-foreground: 210 40% 98%;
    --border: 217.2 32.6% 17.5%;
    --input: 217.2 32.6% 17.5%;
    --ring: 239 84% 67%;
  }
}

@layer base {
  * {
    @apply border-border;
  }
  body {
    @apply bg-background text-foreground;
  }
}
```

### 4. Create Utility Helper

Create `src/lib/utils.ts`:

```typescript
import { clsx, type ClassValue } from 'clsx'
import { twMerge } from 'tailwind-merge'

export function cn(...inputs: ClassValue[]) {
  return twMerge(clsx(inputs))
}
```

---

## Setup Theme Provider

### 1. Create ThemeProvider Component

Create `src/components/providers/theme-provider.tsx`:

```typescript
'use client'

import * as React from 'react'
import { ThemeProvider as NextThemesProvider } from 'next-themes'
import { type ThemeProviderProps } from 'next-themes/dist/types'

export function ThemeProvider({ children, ...props }: ThemeProviderProps) {
  return <NextThemesProvider {...props}>{children}</NextThemesProvider>
}
```

### 2. Wrap Application

Update `src/app/layout.tsx`:

```typescript
import { ThemeProvider } from '@/components/providers/theme-provider'
import './globals.css'

export default function RootLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return (
    <html lang="en" suppressHydrationWarning>
      <body>
        <ThemeProvider
          attribute="class"
          defaultTheme="system"
          enableSystem
          disableTransitionOnChange={false}
          storageKey="todo-theme"
        >
          {children}
        </ThemeProvider>
      </body>
    </html>
  )
}
```

---

## Using the Theme System

### Theme Toggle Component

Create `src/components/theme/mode-toggle.tsx`:

```typescript
'use client'

import * as React from 'react'
import { Moon, Sun } from 'lucide-react'
import { useTheme } from 'next-themes'

export function ModeToggle() {
  const { theme, setTheme } = useTheme()
  const [mounted, setMounted] = React.useState(false)

  // Prevent hydration mismatch
  React.useEffect(() => {
    setMounted(true)
  }, [])

  if (!mounted) {
    return <div className="w-9 h-9" /> // Placeholder
  }

  return (
    <button
      onClick={() => setTheme(theme === 'dark' ? 'light' : 'dark')}
      className="relative w-9 h-9 rounded-lg bg-muted hover:bg-accent transition-colors"
      aria-label="Toggle theme"
    >
      <Sun className="absolute inset-0 m-auto h-5 w-5 rotate-0 scale-100 transition-all dark:-rotate-90 dark:scale-0" />
      <Moon className="absolute inset-0 m-auto h-5 w-5 rotate-90 scale-0 transition-all dark:rotate-0 dark:scale-100" />
    </button>
  )
}
```

### Using Theme in Components

```typescript
'use client'

import { useTheme } from 'next-themes'

export function ThemeAwareComponent() {
  const { theme, resolvedTheme } = useTheme()

  return (
    <div>
      <p>Current theme: {theme}</p>
      <p>Resolved theme: {resolvedTheme}</p>
    </div>
  )
}
```

---

## Using Animations

### Basic Animation Example

```typescript
'use client'

import { motion } from 'framer-motion'

export function AnimatedCard() {
  return (
    <motion.div
      initial={{ opacity: 0, y: 20 }}
      animate={{ opacity: 1, y: 0 }}
      transition={{ duration: 0.3 }}
      className="p-4 bg-card rounded-xl shadow-lg"
    >
      Card content
    </motion.div>
  )
}
```

### List Animation with AnimatePresence

```typescript
'use client'

import { motion, AnimatePresence } from 'framer-motion'

export function AnimatedList({ items }: { items: Array<{ id: string; title: string }> }) {
  return (
    <ul>
      <AnimatePresence mode="popLayout">
        {items.map(item => (
          <motion.li
            key={item.id}
            layout
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            exit={{ opacity: 0, x: -100 }}
            transition={{ duration: 0.3 }}
            className="p-4 mb-2 bg-card rounded-lg"
          >
            {item.title}
          </motion.li>
        ))}
      </AnimatePresence>
    </ul>
  )
}
```

### Reduced Motion Support

```typescript
'use client'

import { motion, useReducedMotion } from 'framer-motion'

export function AccessibleAnimation() {
  const shouldReduceMotion = useReducedMotion()

  return (
    <motion.div
      animate={{ opacity: 1, y: 0 }}
      transition={{
        duration: shouldReduceMotion ? 0 : 0.3,
        type: shouldReduceMotion ? 'tween' : 'spring',
      }}
    >
      Content
    </motion.div>
  )
}
```

---

## Creating Styled Components

### Button Component Example

```typescript
'use client'

import { ButtonHTMLAttributes, forwardRef } from 'react'
import { motion } from 'framer-motion'
import { cn } from '@/lib/utils'

interface ButtonProps extends ButtonHTMLAttributes<HTMLButtonElement> {
  variant?: 'primary' | 'secondary' | 'ghost'
  size?: 'sm' | 'md' | 'lg'
}

export const Button = forwardRef<HTMLButtonElement, ButtonProps>(
  ({ className, variant = 'primary', size = 'md', children, ...props }, ref) => {
    return (
      <motion.button
        ref={ref}
        whileHover={{ scale: 1.02 }}
        whileTap={{ scale: 0.98 }}
        className={cn(
          'rounded-lg font-medium transition-all',
          'focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring',
          {
            'bg-primary text-primary-foreground hover:bg-primary/90': variant === 'primary',
            'bg-secondary text-secondary-foreground hover:bg-secondary/80': variant === 'secondary',
            'hover:bg-accent hover:text-accent-foreground': variant === 'ghost',
          },
          {
            'px-3 py-1.5 text-sm': size === 'sm',
            'px-4 py-2 text-base': size === 'md',
            'px-6 py-3 text-lg': size === 'lg',
          },
          className
        )}
        {...props}
      >
        {children}
      </motion.button>
    )
  }
)

Button.displayName = 'Button'
```

### Card Component Example

```typescript
'use client'

import { HTMLAttributes, forwardRef } from 'react'
import { cn } from '@/lib/utils'

interface CardProps extends HTMLAttributes<HTMLDivElement> {
  variant?: 'default' | 'glass'
}

export const Card = forwardRef<HTMLDivElement, CardProps>(
  ({ className, variant = 'default', children, ...props }, ref) => {
    return (
      <div
        ref={ref}
        className={cn(
          'rounded-xl border shadow-lg',
          {
            'bg-card text-card-foreground': variant === 'default',
            'bg-white/10 dark:bg-black/30 backdrop-blur-md border-white/20': variant === 'glass',
          },
          className
        )}
        {...props}
      >
        {children}
      </div>
    )
  }
)

Card.displayName = 'Card'
```

---

## Testing

### Testing Theme Switching

```typescript
import { render, screen } from '@testing-library/react'
import { ThemeProvider } from 'next-themes'
import { ModeToggle } from '@/components/theme/mode-toggle'

describe('ModeToggle', () => {
  it('toggles theme on click', () => {
    render(
      <ThemeProvider>
        <ModeToggle />
      </ThemeProvider>
    )

    const button = screen.getByRole('button', { name: /toggle theme/i })
    expect(button).toBeInTheDocument()
  })
})
```

### Testing Animations

```typescript
import { render } from '@testing-library/react'
import { AnimatedCard } from '@/components/ui/card'

describe('AnimatedCard', () => {
  it('renders with animation', () => {
    const { container } = render(<AnimatedCard>Content</AnimatedCard>)
    expect(container.firstChild).toHaveStyle({ opacity: 0 })
  })
})
```

---

## Accessibility Testing

### Manual Testing Checklist

- [ ] Test with keyboard navigation (Tab, Enter, Space)
- [ ] Verify focus indicators are visible
- [ ] Test with screen reader (NVDA, JAWS, VoiceOver)
- [ ] Enable "Reduce motion" in OS settings and verify animations are simplified
- [ ] Check color contrast ratios with browser DevTools
- [ ] Test theme switching with system preference changes

### Automated Testing

```bash
# Run Lighthouse accessibility audit
npm run build
npx lighthouse http://localhost:3000 --only-categories=accessibility

# Run axe-core tests
npm install -D @axe-core/playwright
```

---

## Performance Optimization

### Best Practices

1. **Use CSS transforms**: Animate `transform` and `opacity` only
2. **Add will-change hints**: For elements that will animate
3. **Lazy load Framer Motion**: Only import in client components
4. **Memoize components**: Use `React.memo()` for static components
5. **Batch animations**: Group multiple animations into single layout pass

### Performance Monitoring

```typescript
'use client'

import { useEffect } from 'react'

export function PerformanceMonitor() {
  useEffect(() => {
    // Monitor frame rate
    let frameCount = 0
    let lastTime = performance.now()

    function measureFPS() {
      frameCount++
      const currentTime = performance.now()

      if (currentTime >= lastTime + 1000) {
        const fps = Math.round((frameCount * 1000) / (currentTime - lastTime))
        console.log(`FPS: ${fps}`)
        frameCount = 0
        lastTime = currentTime
      }

      requestAnimationFrame(measureFPS)
    }

    measureFPS()
  }, [])

  return null
}
```

---

## Troubleshooting

### FOUC (Flash of Unstyled Content)

**Problem**: Theme flashes on page load

**Solution**: Ensure `suppressHydrationWarning` is on `<html>` tag:

```typescript
<html lang="en" suppressHydrationWarning>
```

### Animations Not Working

**Problem**: Framer Motion animations don't play

**Solution**: Ensure component is marked as client component:

```typescript
'use client'

import { motion } from 'framer-motion'
```

### Theme Not Persisting

**Problem**: Theme resets on page reload

**Solution**: Check localStorage key matches:

```typescript
<ThemeProvider storageKey="todo-theme">
```

### Poor Animation Performance

**Problem**: Animations are janky or slow

**Solution**: Use GPU-accelerated properties:

```typescript
// ✅ Good
<motion.div animate={{ x: 100, opacity: 0.5 }} />

// ❌ Bad
<motion.div animate={{ left: '100px', width: '200px' }} />
```

---

## Resources

- [Framer Motion Documentation](https://www.framer.com/motion/)
- [next-themes Documentation](https://github.com/pacocoursey/next-themes)
- [Tailwind CSS Documentation](https://tailwindcss.com/docs)
- [WCAG Accessibility Guidelines](https://www.w3.org/WAI/WCAG21/quickref/)
- [Web Animations Performance](https://web.dev/animations/)

---

## Next Steps

1. Review the [contracts](./contracts/) for component interfaces
2. Check [data-model.md](./data-model.md) for state management details
3. Read [research.md](./research.md) for technology decisions
4. Run `/sp.tasks` to generate implementation tasks
5. Start implementing with Claude Code

---

**Quickstart Status**: ✅ Complete | Ready for development
