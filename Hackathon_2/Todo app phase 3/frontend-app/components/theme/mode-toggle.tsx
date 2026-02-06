"use client"

import * as React from "react"
import { Moon, Sun } from "lucide-react"
import { useTheme } from "next-themes"
import { motion } from "framer-motion"

export function ModeToggle() {
  const { theme, setTheme } = useTheme()
  const [mounted, setMounted] = React.useState(false)

  // Avoid hydration mismatch
  React.useEffect(() => {
    setMounted(true)
  }, [])

  if (!mounted) {
    return (
      <button
        className="relative inline-flex h-10 w-10 items-center justify-center rounded-lg border border-input bg-background text-sm font-medium transition-colors hover:bg-accent hover:text-accent-foreground focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring disabled:pointer-events-none disabled:opacity-50"
        disabled
      >
        <Sun className="h-5 w-5" />
      </button>
    )
  }

  const isDark = theme === "dark"

  return (
    <motion.button
      onClick={() => setTheme(isDark ? "light" : "dark")}
      className="relative inline-flex h-12 w-12 items-center justify-center rounded-xl bg-gradient-to-br from-primary-500/20 to-secondary-500/20 dark:from-primary-600/30 dark:to-secondary-600/30 border-2 border-primary-400/30 dark:border-primary-500/40 backdrop-blur-sm shadow-lg hover:shadow-xl hover:shadow-primary-500/30 transition-all duration-300"
      whileHover={{ scale: 1.1, rotate: 5 }}
      whileTap={{ scale: 0.95, rotate: -5 }}
      aria-label="Toggle theme"
    >
      <motion.div
        initial={false}
        animate={{
          scale: isDark ? 0 : 1,
          opacity: isDark ? 0 : 1,
          rotate: isDark ? 180 : 0,
        }}
        transition={{ duration: 0.3, ease: "easeOut", type: "spring", stiffness: 200 }}
        className="absolute"
      >
        <Sun className="h-6 w-6 text-primary-600 dark:text-primary-400" />
      </motion.div>
      <motion.div
        initial={false}
        animate={{
          scale: isDark ? 1 : 0,
          opacity: isDark ? 1 : 0,
          rotate: isDark ? 0 : -180,
        }}
        transition={{ duration: 0.3, ease: "easeOut", type: "spring", stiffness: 200 }}
        className="absolute"
      >
        <Moon className="h-6 w-6 text-primary-600 dark:text-primary-400" />
      </motion.div>
    </motion.button>
  )
}
