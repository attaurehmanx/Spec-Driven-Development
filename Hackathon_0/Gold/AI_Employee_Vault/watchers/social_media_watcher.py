"""
Social Media Watcher - Facebook, Instagram, Twitter/X
Gold Tier Implementation
"""
import os
import requests
from pathlib import Path
from datetime import datetime, timedelta
from dotenv import load_dotenv
from base_watcher import BaseWatcher

# Load environment variables
load_dotenv()


class SocialMediaWatcher(BaseWatcher):
    """
    Monitors social media platforms for mentions, messages, and engagement opportunities.
    Uses official APIs for each platform.
    """

    def __init__(self, vault_path: str, platform: str, check_interval: int = 60):
        """
        Args:
            vault_path: Path to Obsidian vault
            platform: 'facebook', 'instagram', or 'twitter'
            check_interval: Check every 1 minute (60 seconds)
        """
        super().__init__(vault_path, check_interval)
        self.platform = platform
        self.platform_folder = self.needs_action / platform
        self.platform_folder.mkdir(parents=True, exist_ok=True)

        # Keywords to monitor
        self.keywords = [
            'urgent', 'help', 'question', 'inquiry', 'pricing',
            'quote', 'service', 'support', 'issue', 'problem'
        ]

        # Track endpoints that are known to fail (to avoid repeated logging)
        self.disabled_endpoints = set()

        # Load API credentials
        self._load_credentials()

    def _load_credentials(self):
        """Load API credentials from environment"""
        if self.platform == 'facebook':
            self.access_token = os.getenv('FACEBOOK_ACCESS_TOKEN')
            self.page_id = os.getenv('FACEBOOK_PAGE_ID')
            self.api_base = 'https://graph.facebook.com/v18.0'
        elif self.platform == 'instagram':
            self.access_token = os.getenv('INSTAGRAM_ACCESS_TOKEN')
            self.account_id = os.getenv('INSTAGRAM_ACCOUNT_ID')
            self.api_base = 'https://graph.facebook.com/v18.0'
        elif self.platform == 'twitter':
            self.bearer_token = os.getenv('TWITTER_BEARER_TOKEN')
            self.api_base = 'https://api.twitter.com/2'
            self.twitter_user_id = os.getenv('TWITTER_USER_ID')

    def check_for_updates(self) -> list:
        """
        Check for new mentions, messages, or engagement opportunities.
        """
        if self.platform == 'facebook':
            return self._check_facebook()
        elif self.platform == 'instagram':
            return self._check_instagram()
        elif self.platform == 'twitter':
            return self._check_twitter()
        return []

    def _check_facebook(self) -> list:
        """
        Check Facebook for new activity using Graph API.
        Monitors: Page posts comments, messages, mentions
        """
        if not self.access_token or not self.page_id:
            self.logger.warning('Facebook credentials not configured')
            return []

        items = []
        try:
            # Check for new comments on page posts
            comments = self._get_page_comments()
            items.extend(comments)

            # Check for new messages
            messages = self._get_page_messages()
            items.extend(messages)

            # Check for mentions
            mentions = self._get_page_mentions()
            items.extend(mentions)

        except Exception as e:
            self.logger.error(f'Error checking Facebook: {e}')

        return items

    def _get_page_comments(self) -> list:
        """Get recent comments on page posts"""
        items = []
        try:
            # Get page posts
            posts_url = f'{self.api_base}/{self.page_id}/posts'
            params = {
                'access_token': self.access_token,
                'limit': 5,
                'fields': 'id,message,created_time,comments.limit(10){id,message,from,created_time}'
            }
            response = requests.get(posts_url, params=params, timeout=30)
            response.raise_for_status()
            posts = response.json().get('data', [])

            for post in posts:
                comments = post.get('comments', {}).get('data', [])
                for comment in comments:
                    if comment['id'] not in self.processed_ids:
                        items.append({
                            'id': comment['id'],
                            'type': 'comment',
                            'from': comment.get('from', {}).get('name', 'Unknown'),
                            'text': comment.get('message', ''),
                            'timestamp': comment.get('created_time', datetime.now().isoformat()),
                            'url': f"https://facebook.com/{comment['id']}",
                            'post_id': post['id']
                        })
                        # Don't add to processed_ids here - will be added after file creation

        except Exception as e:
            self.logger.error(f'Error getting page comments: {e}')

        return items

    def _get_page_messages(self) -> list:
        """Get recent messages from page inbox"""
        items = []

        # Skip if this endpoint is known to fail
        if 'messages' in self.disabled_endpoints:
            return items

        try:
            messages_url = f'{self.api_base}/{self.page_id}/conversations'
            params = {
                'access_token': self.access_token,
                'limit': 10,
                'fields': 'id,messages.limit(5){id,message,from,created_time},updated_time'
            }
            response = requests.get(messages_url, params=params, timeout=30)

            # Handle 403 Forbidden specifically
            if response.status_code == 403:
                self.disabled_endpoints.add('messages')
                self.logger.warning('Facebook messages endpoint requires pages_messaging permission. Skipping messages monitoring.')
                return items

            response.raise_for_status()
            conversations = response.json().get('data', [])

            for conv in conversations:
                messages = conv.get('messages', {}).get('data', [])
                for msg in messages:
                    if msg['id'] not in self.processed_ids:
                        # Check if message is recent (last 5 minutes)
                        msg_time = datetime.fromisoformat(msg['created_time'].replace('Z', '+00:00'))
                        if datetime.now(msg_time.tzinfo) - msg_time < timedelta(minutes=5):
                            items.append({
                                'id': msg['id'],
                                'type': 'message',
                                'from': msg.get('from', {}).get('name', 'Unknown'),
                                'text': msg.get('message', ''),
                                'timestamp': msg.get('created_time', datetime.now().isoformat()),
                                'url': f"https://m.me/{self.page_id}",
                                'conversation_id': conv['id']
                            })
                            # Don't add to processed_ids here - will be added after file creation

        except Exception as e:
            self.logger.error(f'Error getting page messages: {e}')

        return items

    def _get_page_mentions(self) -> list:
        """Get recent mentions/tags of the page"""
        items = []
        try:
            mentions_url = f'{self.api_base}/{self.page_id}/tagged'
            params = {
                'access_token': self.access_token,
                'limit': 10,
                'fields': 'id,message,from,created_time,permalink_url'
            }
            response = requests.get(mentions_url, params=params, timeout=30)
            response.raise_for_status()
            mentions = response.json().get('data', [])

            for mention in mentions:
                if mention['id'] not in self.processed_ids:
                    items.append({
                        'id': mention['id'],
                        'type': 'mention',
                        'from': mention.get('from', {}).get('name', 'Unknown'),
                        'text': mention.get('message', ''),
                        'timestamp': mention.get('created_time', datetime.now().isoformat()),
                        'url': mention.get('permalink_url', '')
                    })
                    # Don't add to processed_ids here - will be added after file creation

        except Exception as e:
            self.logger.error(f'Error getting mentions: {e}')

        return items

    def _check_instagram(self) -> list:
        """
        Check Instagram for new activity using Instagram Graph API.
        Monitors: Comments on ALL media posts
        """
        if not self.access_token or not self.account_id:
            self.logger.warning('Instagram credentials not configured')
            return []

        items = []
        try:
            # Get all comments from all recent media
            comments = self._get_all_media_comments()
            items.extend(comments)

        except Exception as e:
            self.logger.error(f'Error checking Instagram: {e}')

        return items

    def _get_all_media_comments(self) -> list:
        """Get comments from ALL recent media posts - WITH DEDUPLICATION"""
        items = []
        try:
            # First, get all recent media posts
            media_url = f'{self.api_base}/{self.account_id}/media'
            media_params = {
                'access_token': self.access_token,
                'limit': 25,  # Get more posts
                'fields': 'id,caption,media_type,permalink'
            }
            media_response = requests.get(media_url, params=media_params, timeout=30)
            media_response.raise_for_status()
            media_items = media_response.json().get('data', [])

            self.logger.info(f'Found {len(media_items)} media posts to check')
            self.logger.info(f'Already processed {len(self.processed_ids)} comments (will skip)')

            # Now get comments for EACH media post
            for media_item in media_items:
                media_id = media_item['id']
                
                # Fetch comments for this specific media post
                comments_url = f'{self.api_base}/{media_id}/comments'
                params = {
                    'access_token': self.access_token,
                    'limit': 100,  # Get up to 100 comments per post
                    'fields': 'id,text,username,timestamp'
                }
                response = requests.get(comments_url, params=params, timeout=30)
                
                if response.status_code != 200:
                    self.logger.warning(f'Could not fetch comments for media {media_id}: {response.status_code}')
                    continue
                    
                comments = response.json().get('data', [])
                
                for comment in comments:
                    # Skip if already processed (deduplication)
                    if comment['id'] in self.processed_ids:
                        self.logger.debug(f"Skipping already processed comment: {comment['id']}")
                        continue
                    
                    # New comment - add to items
                    self.logger.info(f"  New comment from @{comment.get('username', 'Unknown')}: {comment.get('text', '')[:40]}")
                    items.append({
                        'id': comment['id'],
                        'type': 'comment',
                        'from': comment.get('username', 'Unknown'),
                        'text': comment.get('text', ''),
                        'timestamp': comment.get('timestamp', datetime.now().isoformat()),
                        'url': media_item.get('permalink', ''),
                        'media_id': media_id,
                        'media_caption': media_item.get('caption', '')
                    })

            self.logger.info(f'New comments to process: {len(items)}')

        except Exception as e:
            self.logger.error(f'Error getting Instagram media comments: {e}')

        return items

    def _check_twitter(self) -> list:
        """
        Check Twitter/X for new activity using Twitter API v2.
        Monitors: Mentions, DMs, replies
        """
        if not self.bearer_token or not self.twitter_user_id:
            self.logger.warning('Twitter credentials not configured')
            return []

        items = []
        try:
            # Get mentions
            mentions = self._get_mentions()
            items.extend(mentions)

            # Get DMs (requires elevated access)
            dms = self._get_direct_messages()
            items.extend(dms)

        except Exception as e:
            self.logger.error(f'Error checking Twitter: {e}')

        return items

    def _get_mentions(self) -> list:
        """Get recent mentions using Twitter API v2"""
        items = []
        try:
            mentions_url = f'{self.api_base}/users/{self.twitter_user_id}/mentions'
            headers = {
                'Authorization': f'Bearer {self.bearer_token}',
                'User-Agent': 'GoldTierAIEmployee'
            }
            params = {
                'max_results': 20,
                'tweet.fields': 'created_at,author_id,text,conversation_id,referenced_tweets',
                'expansions': 'author_id',
                'user.fields': 'name,username'
            }
            response = requests.get(mentions_url, headers=headers, params=params, timeout=30)
            response.raise_for_status()
            data = response.json()

            tweets = data.get('data', [])
            users = {u['id']: u for u in data.get('includes', {}).get('users', [])}

            for tweet in tweets:
                if tweet['id'] not in self.processed_ids:
                    author = users.get(tweet['author_id'], {})
                    items.append({
                        'id': tweet['id'],
                        'type': 'mention',
                        'from': f"@{author.get('username', 'Unknown')}",
                        'text': tweet.get('text', ''),
                        'timestamp': tweet.get('created_at', datetime.now().isoformat()),
                        'url': f"https://twitter.com/{author.get('username', '')}/status/{tweet['id']}",
                        'conversation_id': tweet.get('conversation_id', '')
                    })
                    # Don't add to processed_ids here - will be added after file creation

        except Exception as e:
            self.logger.error(f'Error getting Twitter mentions: {e}')

        return items

    def _get_direct_messages(self) -> list:
        """Get recent direct messages (requires elevated API access)"""
        items = []
        try:
            dm_url = f'{self.api_base}/dm_conversations/{self.twitter_user_id}/dm_events'
            headers = {
                'Authorization': f'Bearer {self.bearer_token}',
                'User-Agent': 'GoldTierAIEmployee'
            }
            params = {
                'max_results': 10,
                'dm_event.fields': 'text,created_at,sender_id,dm_conversation_id',
                'expansions': 'sender_id',
                'user.fields': 'name,username'
            }
            response = requests.get(dm_url, headers=headers, params=params, timeout=30)

            if response.status_code == 403:
                self.logger.warning('Twitter DM access requires elevated API access')
                return []

            response.raise_for_status()
            data = response.json()

            events = data.get('data', [])
            users = {u['id']: u for u in data.get('includes', {}).get('users', [])}

            for event in events:
                if event['id'] not in self.processed_ids:
                    sender = users.get(event.get('sender_id'), {})
                    items.append({
                        'id': event['id'],
                        'type': 'direct_message',
                        'from': f"@{sender.get('username', 'Unknown')}",
                        'text': event.get('text', ''),
                        'timestamp': event.get('created_at', datetime.now().isoformat()),
                        'conversation_id': event.get('dm_conversation_id', '')
                    })
                    # Don't add to processed_ids here - will be added after file creation

        except Exception as e:
            self.logger.error(f'Error getting Twitter DMs: {e}')

        return items

    def create_action_file(self, item) -> Path:
        """Create action file for social media item - WITH DEDUPLICATION"""

        item_id = item.get('id', 'unknown')
        item_type = item.get('type', 'mention')

        # Skip if already processed (deduplication)
        if item_id in self.processed_ids:
            self.logger.debug(f"Skipping duplicate item: {item_id}")
            return None

        # Create metadata
        metadata = {
            'platform': self.platform,
            'item_type': item_type,
            'from': item.get('from', 'Unknown'),
            'priority': self._calculate_priority(item),
            'url': item.get('url', '')
        }

        # Create content
        content = self.create_metadata_header(f'{self.platform}_{item_type}', metadata)
        content += f"## {self.platform.title()} {item_type.title()}\n\n"
        content += f"**From**: {item.get('from', 'Unknown')}\n"
        content += f"**Time**: {item.get('timestamp', datetime.now().isoformat())}\n\n"
        content += f"### Content\n{item.get('text', 'No content')}\n\n"

        # Add context for specific types
        if item_type == 'comment':
            content += f"### Context\n"
            content += f"- **Post/Media URL**: {item.get('url', 'N/A')}\n"
            if 'post_id' in item:
                content += f"- **Post ID**: {item['post_id']}\n"
            if 'media_id' in item:
                content += f"- **Media ID**: {item['media_id']}\n"
            if 'media_caption' in item and item['media_caption']:
                content += f"- **Media Caption**: {item['media_caption']}\n"
            if 'conversation_id' in item:
                content += f"- **Conversation ID**: {item['conversation_id']}\n"
            content += "\n"

        content += f"### Suggested Actions\n"
        content += f"- [ ] Review and respond\n"
        content += f"- [ ] Engage with content\n"
        content += f"- [ ] Track for analytics\n"

        # Create file with unique filename including comment ID
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        item_id_short = str(item_id).replace('_', '')[:15]  # First 15 chars of ID
        filename = f'{self.platform.upper()}_{item_type}_{timestamp}_{item_id_short}.md'
        filepath = self.platform_folder / filename

        filepath.write_text(content, encoding='utf-8')
        self.processed_ids.add(item_id)

        # Log with full comment ID and commenter name
        self.logger.info(f'Created action file: {filename} & comment_id: {item_id} & comment from "{item.get("from", "Unknown")}"')

        return filepath

    def _calculate_priority(self, item) -> str:
        """Calculate priority based on content and context"""
        text = item.get('text', '').lower()

        # Check for urgent keywords
        urgent_keywords = ['urgent', 'asap', 'emergency', 'help', 'issue', 'problem']
        if any(keyword in text for keyword in urgent_keywords):
            return 'high'

        # Check for business keywords
        business_keywords = ['pricing', 'quote', 'service', 'inquiry', 'question']
        if any(keyword in text for keyword in business_keywords):
            return 'medium'

        return 'low'


class FacebookWatcher(SocialMediaWatcher):
    """Facebook-specific watcher"""
    def __init__(self, vault_path: str):
        super().__init__(vault_path, 'facebook', check_interval=60)


class InstagramWatcher(SocialMediaWatcher):
    """Instagram-specific watcher"""
    def __init__(self, vault_path: str):
        super().__init__(vault_path, 'instagram', check_interval=60)


class TwitterWatcher(SocialMediaWatcher):
    """Twitter/X-specific watcher"""
    def __init__(self, vault_path: str):
        super().__init__(vault_path, 'twitter', check_interval=60)


if __name__ == '__main__':
    import sys

    if len(sys.argv) < 3:
        print("Usage: python social_media_watcher.py <vault_path> <platform>")
        print("Platforms: facebook, instagram, twitter")
        sys.exit(1)

    vault_path = sys.argv[1]
    platform = sys.argv[2].lower()

    if platform == 'facebook':
        watcher = FacebookWatcher(vault_path)
    elif platform == 'instagram':
        watcher = InstagramWatcher(vault_path)
    elif platform == 'twitter':
        watcher = TwitterWatcher(vault_path)
    else:
        print(f"Unknown platform: {platform}")
        sys.exit(1)

    watcher.run()
