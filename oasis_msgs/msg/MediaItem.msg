################################################################################
#
#  Copyright (C) 2022-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Now Playing Media Item
#

# The header and timestamp of the media item frame
std_msgs/Header header

#
# The media type, or empty for "none"
#
# The following are valid types:
#
#   * music
#   * artist
#   * album
#   * song
#   * video
#   * set
#   * musicvideo
#   * movie
#   * tvshow
#   * season
#   * episode
#   * oversion
#
string media_type

# The media title, or empty if unknown
string title

# The release year, or 0 if unknown
uint32 year

# The poster URL, or empty if unknown
string poster_url

# The current playtime, in seconds, or 0 if unknown
uint64 playtime_secs
