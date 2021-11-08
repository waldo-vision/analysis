from parsers.csgo_parser import CSGOParser

# Do NOT set parse_rate to 1 or your PC will have a bad time.
# Or set it to 1 to get more data.
parser = CSGOParser(parse_rate=64)
parser.parse('fixtures/default.dem')
data = parser.get_parsed_data()

print(data.keys())
frames = data["gameRounds"][0]["frames"]

print(f'roundEndReason: {data["gameRounds"][0]["roundEndReason"]}')
print(f'endOfficialTick: {data["gameRounds"][0]["endOfficialTick"]}')

for i in range(len(frames)):
    print(frames[i]["tick"])