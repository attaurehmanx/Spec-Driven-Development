# Data Model: UI/UX Modernization

**Feature**: 008-ui-ux-modernization
**Date**: 2026-02-02
**Purpose**: Define state management and data structures for theme and animation systems

---

## Overview

This feature introduces minimal state management focused on presentation layer concerns. No database persistence is required. All state is either transient (animation state) or persisted in browser storage (theme preference).

---

## 1. Theme State

### Storage Location
Browser `localStorage` with key `todo-theme`

### Data Structure
```typescript
type ThemeMode = 'light' | 'dark' | 'system'

interface ThemeState {
  theme: ThemeMode
  systemTheme: 'light' | 'dark' // Detected from OS
  resolvedTheme: 'light' | 'dark' // Actual applied theme
}
```

### State Management
- **Library**: next-themes
- **Provider**: ThemeProvider component wrapping app root
- **Access**: `useTheme()` hook in client components

### Persistence Strategy
- Automatically persisted to localStorage by next-themes
- Key: `todo-theme`
- Value: `'light' | 'dark' | 'system'`
- Synced across browser tabs via storage events

### Initialization Flow
1. On app load, next-themes checks localStorage for saved preference
2. If no preference found, defaults to `'system'`
3. If `'system'`, detects OS preference via `prefers-color-scheme` media query
4. Applies theme class to `<html>` element before hydration (prevents FOUC)
5. Listens for system theme changes and updates automatically

### State Transitions
```
User Action → Theme Change → localStorage Update → Class Toggle → CSS Variables Update
```

### Example Usage
```typescript
'use client'

import { useTheme } from 'next-themes'

export function ThemeAwareComponent() {
  const { theme, setTheme, systemTheme, resolvedTheme } = useTheme()

  return (
    <div>
      <p>Current theme: {theme}</p>
      <p>System theme: {systemTheme}</p>
      <p>Resolved theme: {resolvedTheme}</p>
      <button onClick={() => setTheme('dark')}>Dark</button>
      <button onClick={() => setTheme('light')}>Light</button>
      <button onClick={() => setTheme('system')}>System</button>
    </div>
  )
}
```

---

## 2. Animation State

### Storage Location
Transient React component state (not persisted)

### Data Structure
```typescript
interface AnimationState {
  isAnimating: boolean
  animationId: string | null
  animationQueue: AnimationTask[]
}

interface AnimationTask {
  id: string
  type: 'fade-in' | 'slide-up' | 'slide-away' | 'strikethrough' | 'layout'
  targetId: string
  duration: number
  delay?: number
}
```

### State Management
- **Library**: Framer Motion (internal state management)
- **Access**: Motion component props and variants
- **Lifecycle**: Created on mount, destroyed on unmount

### Animation Coordination
For batch operations (multiple tasks added/removed simultaneously):

```typescript
interface AnimationQueue {
  tasks: AnimationTask[]
  isProcessing: boolean
  currentIndex: number
}

// Example: Batch task deletion
const animationQueue = {
  tasks: [
    { id: '1', type: 'slide-away', targetId: 'task-1', duration: 300 },
    { id: '2', type: 'slide-away', targetId: 'task-2', duration: 300, delay: 100 },
    { id: '3', type: 'layout', targetId: 'task-list', duration: 400, delay: 200 },
  ],
  isProcessing: true,
  currentIndex: 0,
}
```

### Framer Motion State
Framer Motion manages animation state internally through:
- `AnimatePresence`: Tracks mounted/unmounted components
- `LayoutGroup`: Coordinates layout animations across siblings
- `useAnimation()`: Programmatic animation control

### Example Usage
```typescript
'use client'

import { motion, AnimatePresence } from 'framer-motion'

export function AnimatedTaskList({ tasks }) {
  return (
    <AnimatePresence mode="popLayout">
      {tasks.map(task => (
        <motion.li
          key={task.id}
          layout
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          exit={{ opacity: 0, x: -100 }}
          transition={{ duration: 0.3 }}
        >
          {task.title}
        </motion.li>
      ))}
    </AnimatePresence>
  )
}
```

---

## 3. Reduced Motion Preference

### Storage Location
Browser API (not stored, queried on demand)

### Data Structure
```typescript
interface MotionPreference {
  prefersReducedMotion: boolean
}
```

### Detection Method
```typescript
// Via media query
const prefersReducedMotion = window.matchMedia(
  '(prefers-reduced-motion: reduce)'
).matches

// Via Framer Motion hook
import { useReducedMotion } from 'framer-motion'

function Component() {
  const shouldReduceMotion = useReducedMotion()
  // Returns true if user prefers reduced motion
}
```

### Impact on Animations
When `prefersReducedMotion === true`:
- Animation durations set to 0 (instant transitions)
- Complex animations simplified or disabled
- Layout shifts still animated (accessibility requirement)
- Focus indicators remain visible

### Example Implementation
```typescript
'use client'

import { useReducedMotion } from 'framer-motion'

export function AccessibleAnimation() {
  const shouldReduceMotion = useReducedMotion()

  return (
    <motion.div
      animate={{ opacity: 1, y: 0 }}
      transition={{
        duration: shouldReduceMotion ? 0 : 0.3,
        type: shouldReduceMotion ? 'tween' : 'spring',
      }}
    />
  )
}
```

---

## 4. Design Tokens (CSS Custom Properties)

### Storage Location
CSS (`:root` and `.dark` selectors)

### Data Structure
```css
:root {
  /* Colors (HSL format for easy manipulation) */
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

  /* Spacing */
  --radius: 0.75rem;

  /* Shadows */
  --shadow-sm: 0 1px 2px 0 rgb(0 0 0 / 0.05);
  --shadow-md: 0 4px 6px -1px rgb(0 0 0 / 0.1);
  --shadow-lg: 0 10px 15px -3px rgb(0 0 0 / 0.1);

  /* Animation timing */
  --duration-fast: 150ms;
  --duration-normal: 300ms;
  --duration-slow: 400ms;
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
```

### TypeScript Type Definitions
```typescript
type ColorToken =
  | 'background'
  | 'foreground'
  | 'card'
  | 'card-foreground'
  | 'primary'
  | 'primary-foreground'
  | 'muted'
  | 'muted-foreground'
  | 'accent'
  | 'accent-foreground'
  | 'border'
  | 'input'
  | 'ring'

type ShadowToken = 'shadow-sm' | 'shadow-md' | 'shadow-lg'
type DurationToken = 'duration-fast' | 'duration-normal' | 'duration-slow'
```

---

## 5. Component State (Internal)

### Button Component State
```typescript
interface ButtonState {
  isHovered: boolean
  isPressed: boolean
  isFocused: boolean
  isLoading: boolean
}
```

### Card Component State
```typescript
interface CardState {
  isHovered: boolean
  variant: 'default' | 'glass' | 'elevated'
}
```

### Input Component State
```typescript
interface InputState {
  isFocused: boolean
  hasError: boolean
  value: string
}
```

### Chat Typing Indicator State
```typescript
interface TypingIndicatorState {
  isVisible: boolean
  dots: Array<{ id: number; delay: number }>
}
```

---

## State Flow Diagrams

### Theme Switching Flow
```
User clicks toggle
  ↓
setTheme('dark') called
  ↓
next-themes updates localStorage
  ↓
next-themes toggles .dark class on <html>
  ↓
CSS custom properties update
  ↓
All components re-render with new theme
  ↓
Transition animations play (if enabled)
```

### Task Animation Flow
```
User adds task via chat
  ↓
Task added to React state
  ↓
Component re-renders with new task
  ↓
AnimatePresence detects new element
  ↓
initial animation plays (fade-in + slide-up)
  ↓
Layout animation adjusts sibling positions
  ↓
Animation completes, state stabilizes
```

---

## Data Validation

### Theme Preference Validation
```typescript
const VALID_THEMES = ['light', 'dark', 'system'] as const

function isValidTheme(value: unknown): value is ThemeMode {
  return typeof value === 'string' && VALID_THEMES.includes(value as ThemeMode)
}

// Usage in localStorage read
const savedTheme = localStorage.getItem('todo-theme')
const theme = isValidTheme(savedTheme) ? savedTheme : 'system'
```

### Animation Duration Validation
```typescript
function validateDuration(duration: number): number {
  // Clamp between 0ms and 1000ms
  return Math.max(0, Math.min(duration, 1000))
}
```

---

## Performance Considerations

### State Update Frequency
- **Theme changes**: Infrequent (user-initiated)
- **Animation state**: Frequent during animations (60 FPS)
- **Reduced motion detection**: Once on mount, cached

### Optimization Strategies
1. **Memoization**: Use `React.memo()` for components that don't need theme updates
2. **CSS Variables**: Theme changes don't require React re-renders
3. **Animation Batching**: Group multiple animations into single layout pass
4. **Lazy Loading**: Load Framer Motion only for interactive components

---

## Testing Strategy

### Theme State Tests
- Theme persistence across page reloads
- System theme detection accuracy
- Theme toggle functionality
- FOUC prevention

### Animation State Tests
- Animation trigger conditions
- Animation completion callbacks
- Reduced motion preference handling
- Animation queue processing

### Integration Tests
- Theme + animation coordination
- Multiple simultaneous animations
- State cleanup on unmount

---

**Data Model Status**: ✅ Complete | All state structures defined and documented
