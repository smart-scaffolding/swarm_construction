import os
import time
# os.system("""
# osascript -e '
# tell application "QuickTime Player"
# set newScreenRecording to new screen recording
# tell newScreenRecording
#          start
#          delay 3
#          stop
#      end tell
#      tell last item of documents
#          close
#      end tell
#  end tell'
#  """)

os.system("""
osascript -e '
tell application "QuickTime Player"
set newScreenRecording to new screen recording
tell newScreenRecording
         start
     end tell
 end tell'
 """)

time.sleep(3)
os.system("""
osascript -e '
tell application "QuickTime Player"
	stop document "screen recording"
end tell'
""")