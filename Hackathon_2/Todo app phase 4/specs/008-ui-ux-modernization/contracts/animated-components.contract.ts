/**
 * Animated Components Contract
 *
 * Defines interfaces for animated UI components using Framer Motion.
 * These components provide smooth animations and transitions while maintaining
 * accessibility through reduced motion support.
 */

import { ButtonHTMLAttributes, HTMLAttributes, InputHTMLAttributes, ReactNode } from 'react'
import { Variant, Transition } from 'framer-motion'

/**
 * Button Component
 */

export type ButtonVariant = 'primary' | 'secondary' | 'ghost' | 'destructive'
export type ButtonSize = 'sm' | 'md' | 'lg'

export interface AnimatedButtonProps extends ButtonHTMLAttributes<HTMLButtonElement> {
  /**
   * Visual style variant
   * @default 'primary'
   */
  variant?: ButtonVariant

  /**
   * Button size
   * @default 'md'
   */
  size?: ButtonSize

  /**
   * Show loading spinner and disable interaction
   * @default false
   */
  isLoading?: boolean

  /**
   * Icon to display before text
   */
  leftIcon?: ReactNode

  /**
   * Icon to display after text
   */
  rightIcon?: ReactNode

  /**
   * Full width button
   * @default false
   */
  fullWidth?: boolean
}

/**
 * Card Component
 */

export type CardVariant = 'default' | 'glass' | 'elevated'
export type CardPadding = 'none' | 'sm' | 'md' | 'lg'

export interface AnimatedCardProps extends HTMLAttributes<HTMLDivElement> {
  /**
   * Visual style variant
   * @default 'default'
   */
  variant?: CardVariant

  /**
   * Internal padding
   * @default 'md'
   */
  padding?: CardPadding

  /**
   * Enable hover effect
   * @default false
   */
  hoverable?: boolean

  /**
   * Enable click interaction
   * @default false
   */
  clickable?: boolean

  /**
   * Card header content
   */
  header?: ReactNode

  /**
   * Card footer content
   */
  footer?: ReactNode
}

/**
 * Input Component
 */

export type InputSize = 'sm' | 'md' | 'lg'

export interface AnimatedInputProps extends Omit<InputHTMLAttributes<HTMLInputElement>, 'size'> {
  /**
   * Input label
   */
  label?: string

  /**
   * Error message to display
   */
  error?: string

  /**
   * Helper text to display below input
   */
  helperText?: string

  /**
   * Icon to display inside input (left side)
   */
  icon?: ReactNode

  /**
   * Input size
   * @default 'md'
   */
  size?: InputSize

  /**
   * Full width input
   * @default false
   */
  fullWidth?: boolean
}

/**
 * Task List Item Component
 */

export interface AnimatedTaskItemProps {
  /**
   * Task ID (used as motion key)
   */
  id: string

  /**
   * Task title
   */
  title: string

  /**
   * Task description
   */
  description?: string

  /**
   * Completion status
   */
  completed: boolean

  /**
   * Callback when task is clicked
   */
  onClick?: () => void

  /**
   * Callback when completion status changes
   */
  onToggleComplete?: () => void

  /**
   * Callback when task is deleted
   */
  onDelete?: () => void
}

/**
 * Chat Message Component
 */

export type MessageRole = 'user' | 'assistant'

export interface AnimatedMessageProps {
  /**
   * Message ID (used as motion key)
   */
  id: string

  /**
   * Message role (determines styling)
   */
  role: MessageRole

  /**
   * Message content
   */
  content: string

  /**
   * Timestamp
   */
  timestamp?: Date

  /**
   * Show typing indicator (for assistant messages)
   */
  isTyping?: boolean
}

/**
 * Typing Indicator Component
 */

export interface TypingIndicatorProps {
  /**
   * Show/hide the indicator
   */
  visible: boolean

  /**
   * Custom label text
   * @default 'AI is thinking...'
   */
  label?: string
}

/**
 * Mode Toggle Component
 */

export type ModeToggleVariant = 'icon' | 'button' | 'dropdown'

export interface ModeToggleProps {
  /**
   * Visual variant
   * @default 'icon'
   */
  variant?: ModeToggleVariant

  /**
   * Additional CSS classes
   */
  className?: string

  /**
   * Show label text
   * @default false
   */
  showLabel?: boolean
}

/**
 * Animation Variants
 *
 * Reusable animation configurations for consistent motion design
 */

export const AnimationVariants = {
  /**
   * Fade in animation
   */
  fadeIn: {
    initial: { opacity: 0 },
    animate: { opacity: 1 },
    exit: { opacity: 0 },
  } as const,

  /**
   * Slide up animation
   */
  slideUp: {
    initial: { opacity: 0, y: 20 },
    animate: { opacity: 1, y: 0 },
    exit: { opacity: 0, y: -20 },
  } as const,

  /**
   * Slide away (for deletions)
   */
  slideAway: {
    initial: { opacity: 1, x: 0 },
    animate: { opacity: 1, x: 0 },
    exit: { opacity: 0, x: -100 },
  } as const,

  /**
   * Scale animation
   */
  scale: {
    initial: { scale: 0.95, opacity: 0 },
    animate: { scale: 1, opacity: 1 },
    exit: { scale: 0.95, opacity: 0 },
  } as const,

  /**
   * Strikethrough animation (for completed tasks)
   */
  strikethrough: {
    initial: { scaleX: 0 },
    animate: { scaleX: 1 },
  } as const,
}

/**
 * Animation Transitions
 *
 * Reusable transition configurations
 */

export const AnimationTransitions = {
  /**
   * Fast transition (150ms)
   */
  fast: {
    duration: 0.15,
    ease: 'easeInOut',
  } as Transition,

  /**
   * Normal transition (300ms)
   */
  normal: {
    duration: 0.3,
    ease: 'easeInOut',
  } as Transition,

  /**
   * Slow transition (400ms)
   */
  slow: {
    duration: 0.4,
    ease: 'easeOut',
  } as Transition,

  /**
   * Spring transition (bouncy)
   */
  spring: {
    type: 'spring',
    stiffness: 300,
    damping: 30,
  } as Transition,

  /**
   * Layout transition (for reordering)
   */
  layout: {
    type: 'spring',
    stiffness: 500,
    damping: 50,
  } as Transition,
}

/**
 * Example usage:
 *
 * ```tsx
 * // Button
 * <AnimatedButton
 *   variant="primary"
 *   size="md"
 *   isLoading={isSubmitting}
 *   leftIcon={<PlusIcon />}
 * >
 *   Add Task
 * </AnimatedButton>
 *
 * // Card
 * <AnimatedCard
 *   variant="glass"
 *   padding="lg"
 *   hoverable
 *   header={<h3>Card Title</h3>}
 * >
 *   Card content
 * </AnimatedCard>
 *
 * // Input
 * <AnimatedInput
 *   label="Task Title"
 *   placeholder="Enter task title..."
 *   error={errors.title}
 *   icon={<SearchIcon />}
 * />
 *
 * // Task Item with animation
 * <motion.div
 *   variants={AnimationVariants.slideUp}
 *   transition={AnimationTransitions.normal}
 * >
 *   <AnimatedTaskItem
 *     id={task.id}
 *     title={task.title}
 *     completed={task.completed}
 *     onToggleComplete={handleToggle}
 *   />
 * </motion.div>
 * ```
 */
