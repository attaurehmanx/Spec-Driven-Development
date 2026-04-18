#!/usr/bin/env python3
"""
Social Media Summary Generator
Generates daily/weekly reports with engagement metrics from social media platforms
"""

import os
import json
import requests
from datetime import datetime, timedelta
from pathlib import Path
import re
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class SocialMediaSummaryGenerator:
    def __init__(self, vault_path, period='daily'):
        self.vault_path = Path(vault_path)
        self.period = period
        self.fb_token = os.getenv('FACEBOOK_ACCESS_TOKEN')
        self.ig_token = os.getenv('INSTAGRAM_ACCESS_TOKEN')
        self.twitter_token = os.getenv('TWITTER_BEARER_TOKEN')

        # Create necessary directories
        (self.vault_path / 'Briefings').mkdir(exist_ok=True)
        (self.vault_path / 'Logs').mkdir(exist_ok=True)

    def parse_log(self):
        """Parse social_media_log.txt"""
        log_file = self.vault_path / 'social_media_log.txt'
        posts = []

        if not log_file.exists():
            print(f"Log file not found: {log_file}")
            return posts

        with open(log_file, 'r', encoding='utf-8') as f:
            content = f.read()
            entries = content.split('---')

            for entry in entries:
                if entry.strip():
                    post = {}
                    for line in entry.strip().split('\n'):
                        if ':' in line:
                            key, value = line.split(':', 1)
                            post[key.strip()] = value.strip()
                    if post:  # Only add non-empty posts
                        posts.append(post)

        return posts

    def read_posted_files(self):
        """Read all files in Posted/ folder"""
        posted_dir = self.vault_path / 'Posted'
        files = []

        if not posted_dir.exists():
            print(f"Posted directory not found: {posted_dir}")
            return files

        for file_path in posted_dir.glob('*.md'):
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    # Parse frontmatter
                    if content.startswith('---'):
                        parts = content.split('---', 2)
                        if len(parts) >= 3:
                            metadata = {}
                            for line in parts[1].strip().split('\n'):
                                if ':' in line:
                                    key, value = line.split(':', 1)
                                    metadata[key.strip()] = value.strip()

                            files.append({
                                'file': file_path.name,
                                'metadata': metadata,
                                'content': parts[2].strip()
                            })
            except Exception as e:
                print(f"Error reading {file_path}: {e}")

        return files

    def fetch_facebook_metrics(self, post_id):
        """Fetch real engagement metrics from Facebook"""
        if not self.fb_token:
            print("Facebook access token not found")
            return None

        url = f"https://graph.facebook.com/v18.0/{post_id}"
        params = {
            'fields': 'likes.summary(true),comments.summary(true),shares,reactions.summary(true)',
            'access_token': self.fb_token
        }

        try:
            response = requests.get(url, params=params)
            response.raise_for_status()
            data = response.json()

            # Get insights (impressions, reach)
            insights_url = f"https://graph.facebook.com/v18.0/{post_id}/insights"
            insights_params = {
                'metric': 'post_impressions,post_engaged_users,post_reach',
                'access_token': self.fb_token
            }
            insights_response = requests.get(insights_url, params=insights_params)
            insights_data = insights_response.json() if insights_response.ok else {}

            return {
                'likes': data.get('likes', {}).get('summary', {}).get('total_count', 0),
                'comments': data.get('comments', {}).get('summary', {}).get('total_count', 0),
                'shares': data.get('shares', {}).get('count', 0),
                'reactions': data.get('reactions', {}).get('summary', {}).get('total_count', 0),
                'impressions': self._extract_insight(insights_data, 'post_impressions'),
                'reach': self._extract_insight(insights_data, 'post_reach'),
                'engaged_users': self._extract_insight(insights_data, 'post_engaged_users')
            }
        except Exception as e:
            print(f"Error fetching Facebook metrics for {post_id}: {e}")
            return None

    def _extract_insight(self, insights_data, metric_name):
        """Extract specific metric from insights response"""
        if 'data' in insights_data:
            for metric in insights_data['data']:
                if metric.get('name') == metric_name:
                    values = metric.get('values', [])
                    if values:
                        return values[0].get('value', 0)
        return 0

    def calculate_engagement_rate(self, metrics):
        """Calculate engagement rate"""
        if not metrics or metrics.get('impressions', 0) == 0:
            return 0

        total_engagement = (
            metrics.get('likes', 0) +
            metrics.get('comments', 0) +
            metrics.get('shares', 0)
        )

        return (total_engagement / metrics['impressions']) * 100

    def filter_by_period(self, posts):
        """Filter posts by period (daily/weekly)"""
        now = datetime.now()

        if self.period == 'daily':
            cutoff = now - timedelta(days=1)
        elif self.period == 'weekly':
            cutoff = now - timedelta(weeks=1)
        else:
            cutoff = now - timedelta(days=1)

        filtered = []
        for post in posts:
            posted_str = post.get('Posted', '')
            try:
                # Handle different datetime formats
                posted_str = posted_str.replace('Z', '+00:00')
                posted_date = datetime.fromisoformat(posted_str)
                if posted_date >= cutoff:
                    filtered.append(post)
            except Exception as e:
                print(f"Error parsing date '{posted_str}': {e}")
                continue

        return filtered

    def generate_report(self):
        """Generate the complete report"""
        print(f"Generating {self.period} social media summary...")

        # Parse log and files
        log_posts = self.parse_log()
        posted_files = self.read_posted_files()

        print(f"Found {len(log_posts)} posts in log")

        # Filter by period
        filtered_posts = self.filter_by_period(log_posts)
        print(f"Filtered to {len(filtered_posts)} posts in {self.period} period")

        # Fetch metrics for each post
        enriched_posts = []
        for post in filtered_posts:
            platform = post.get('Platform', '').lower()
            post_id = post.get('Post ID', '')

            print(f"Fetching metrics for {platform} post: {post_id}")

            if platform == 'facebook' and post_id:
                metrics = self.fetch_facebook_metrics(post_id)
                if metrics:
                    post['metrics'] = metrics
                    post['engagement_rate'] = self.calculate_engagement_rate(metrics)
                    enriched_posts.append(post)
                else:
                    # Add post without metrics
                    post['metrics'] = {}
                    post['engagement_rate'] = 0
                    enriched_posts.append(post)

        # Generate report markdown
        report = self._build_report_markdown(enriched_posts)

        # Save report
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        report_file = self.vault_path / 'Briefings' / f'social_media_summary_{self.period}_{timestamp}.md'

        with open(report_file, 'w', encoding='utf-8') as f:
            f.write(report)

        print(f"Report generated: {report_file}")
        return report_file

    def _build_report_markdown(self, posts):
        """Build the markdown report"""
        # Calculate aggregates
        total_posts = len(posts)
        total_impressions = sum(p.get('metrics', {}).get('impressions', 0) for p in posts)
        total_engagement = sum(
            p.get('metrics', {}).get('likes', 0) +
            p.get('metrics', {}).get('comments', 0) +
            p.get('metrics', {}).get('shares', 0)
            for p in posts
        )
        avg_engagement_rate = sum(p.get('engagement_rate', 0) for p in posts) / total_posts if total_posts > 0 else 0

        # Find best post
        best_post = max(posts, key=lambda p: p.get('engagement_rate', 0)) if posts else None

        # Build report
        now = datetime.now().strftime('%Y-%m-%dT%H:%M:%SZ')
        date_range = self._get_date_range()

        report = f"""---
type: social_media_summary
period: {self.period}
generated: {now}
platforms: [facebook]
total_posts: {total_posts}
---

# Social Media Performance Report
**Period**: {self.period.capitalize()} - {date_range}
**Generated**: {now}

## Executive Summary

📊 **Overview**
- Total Posts: {total_posts}
- Total Impressions: {total_impressions:,}
- Total Engagement: {total_engagement:,}
- Average Engagement Rate: {avg_engagement_rate:.2f}%

"""

        if best_post:
            report += f"""🎯 **Key Highlights**
- Best performing platform: Facebook
- Top post: {best_post.get('URL', 'N/A')} ({best_post.get('engagement_rate', 0):.2f}% engagement)
- Engagement trend: {'Up' if avg_engagement_rate > 5 else 'Stable'}

"""

        report += f"""## Platform Breakdown

### Facebook
- Posts: {total_posts}
- Total Reach: {sum(p.get('metrics', {}).get('reach', 0) for p in posts):,}
- Total Engagement: {total_engagement:,}
- Avg Engagement Rate: {avg_engagement_rate:.2f}%

"""

        if best_post:
            metrics = best_post.get('metrics', {})
            report += f"""**Top Post:**
- URL: {best_post.get('URL', 'N/A')}
- Posted: {best_post.get('Posted', 'N/A')}
- Metrics: {metrics.get('likes', 0)} likes, {metrics.get('comments', 0)} comments, {metrics.get('shares', 0)} shares
- Engagement Rate: {best_post.get('engagement_rate', 0):.2f}%

"""

        report += """## Detailed Post Performance

| Posted | Impressions | Likes | Comments | Shares | Engagement Rate | URL |
|--------|-------------|-------|----------|--------|-----------------|-----|
"""

        for post in posts:
            metrics = post.get('metrics', {})
            posted_date = post.get('Posted', 'N/A')[:10] if post.get('Posted') else 'N/A'
            report += f"| {posted_date} | {metrics.get('impressions', 0):,} | {metrics.get('likes', 0)} | {metrics.get('comments', 0)} | {metrics.get('shares', 0)} | {post.get('engagement_rate', 0):.2f}% | {post.get('URL', 'N/A')} |\n"

        report += f"""

## Recommendations

1. **Content Strategy**
   - {'Continue current approach - engagement is strong' if avg_engagement_rate > 5 else 'Consider more engaging content formats (videos, polls, questions)'}

2. **Posting Schedule**
   - Analyze best performing post times and schedule more content during those windows

3. **Platform Focus**
   - Facebook is performing {'well' if avg_engagement_rate > 5 else 'moderately'} - {'maintain' if avg_engagement_rate > 5 else 'increase'} posting frequency

## Action Items

- [ ] Review top performing post and replicate successful elements
- [ ] Respond to all comments on recent posts
- [ ] Schedule next batch of content for optimal times

---

**Next Report**: {self._get_next_report_date()}
**Generated by**: AI Employee Social Media Analytics
"""

        return report

    def _get_date_range(self):
        """Get human-readable date range"""
        now = datetime.now()
        if self.period == 'daily':
            return now.strftime('%Y-%m-%d')
        elif self.period == 'weekly':
            week_ago = now - timedelta(weeks=1)
            return f"{week_ago.strftime('%Y-%m-%d')} to {now.strftime('%Y-%m-%d')}"
        return now.strftime('%Y-%m-%d')

    def _get_next_report_date(self):
        """Get next report date"""
        now = datetime.now()
        if self.period == 'daily':
            next_date = now + timedelta(days=1)
        elif self.period == 'weekly':
            next_date = now + timedelta(weeks=1)
        else:
            next_date = now + timedelta(days=1)
        return next_date.strftime('%Y-%m-%d')


if __name__ == '__main__':
    import sys

    vault_path = sys.argv[1] if len(sys.argv) > 1 else 'AI_Employee_Vault/vault'
    period = sys.argv[2] if len(sys.argv) > 2 else 'daily'

    generator = SocialMediaSummaryGenerator(vault_path, period)
    report_file = generator.generate_report()
    print(f"\nReport generated successfully: {report_file}")
