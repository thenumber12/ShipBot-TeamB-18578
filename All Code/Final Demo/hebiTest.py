import hebi
from time import sleep
lookup = hebi.Lookup()
# Give the Lookup process 2 seconds to discover modules
sleep(2)
print('Modules found on network:')
for entry in lookup.entrylist:
  print(f{entry.family} | {entry.name})
