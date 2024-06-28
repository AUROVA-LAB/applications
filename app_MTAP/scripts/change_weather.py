import carla
import argparse
from utils import *

class ChangeWeather(object):
    def __init__(self, carla_world):
        self.world = carla_world
        self._weather_presets = find_weather_presets()
        self._weather_index = 0

        
    def next(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.world.set_weather(preset[0])

    def set(self, id):
        if id < 0 or id >= len(self._weather_presets):
            print("Invalid weather id.")
            return
        self._weather_index = id
        preset = self._weather_presets[self._weather_index]
        self.world.set_weather(preset[0])

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-t', '--timeout',
        metavar='P',
        default=500,
        help='timeout waiting response from neural network server')
    argparser.add_argument(
        '--weather',
        metavar='P',
        default=None,
        help='Weather id. If not set, the program is interactive.')

    args = argparser.parse_args()
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2000.0)

        sim_world = client.get_world()
        sim_world.wait_for_tick()
        weather = ChangeWeather(sim_world)
        previous_command="exit"
        if args.weather is not None:
            try:
                id=int(args.weather)
                weather.set(id)
            except ValueError:
                print("Incorrect weather id. Exiting...")
                return
        else:
            print("USAGE:\nn\t: Change to next weather.\np\t: Change to previous weather.\n"+
                "id\t: Change to the weather associated to the id (last id is {}).\n\n".format(len(weather._weather_presets)-1)+
                "Empty input to repeat previous command. Otherwise to close the program.")
            command = input("Enter command ")
            while True:
                if command=="": command=previous_command
                previous_command=command
                try:
                    id = int(command)
                    weather.set(id)
                except ValueError:
                    if command=="n":
                        weather.next()
                    elif command=="p":
                        weather.next(True)
                    else:
                        return
                command = input("Enter command (Current weather {}) ".format(weather._weather_index))

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')



if __name__ == '__main__':

    main()