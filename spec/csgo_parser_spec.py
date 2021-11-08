from expects import be_a, expect
from mamba import _it, description, it
from parsers.csgo_parser import CSGOParser
from parsers.replay_parser import ReplayParser

with description('CS:GO Parser') as self:
    with it('assigns a CSGOParser to a variable'):
        parser = CSGOParser()
        expect(parser).to(be_a(CSGOParser))
        expect(parser).to(be_a(ReplayParser))

    with it('successfully parses a valid CS:GO replay'):
        parser = CSGOParser()
        parser.parse('fixtures/default.dem')
        expect(parser.get_parsed_data()).to(be_a(dict))
