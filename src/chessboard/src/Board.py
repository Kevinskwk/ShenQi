from random import randint

W = 9
H = 10

class Board:
    def __init__(self):
        self.tiles = [[' ',' ',' ',' ',' ',' ',' ',' ',' '],
                      [' ',' ',' ',' ',' ',' ',' ',' ',' '],
                      [' ',' ',' ',' ',' ',' ',' ',' ',' '],
                      [' ',' ',' ',' ',' ',' ',' ',' ',' '],
                      [' ',' ',' ',' ',' ',' ',' ',' ',' '],
                      [' ',' ',' ',' ',' ',' ',' ',' ',' '],
                      [' ',' ',' ',' ',' ',' ',' ',' ',' '],
                      [' ',' ',' ',' ',' ',' ',' ',' ',' '],
                      [' ',' ',' ',' ',' ',' ',' ',' ',' '],
                      [' ',' ',' ',' ',' ',' ',' ',' ',' ']]
        self.turn = 0  # 0: start/finish, 1: player1, 2: player2
        self.moves = 0  # number of moves
        self.winner = -1  # -1: not finished, 0: draw, 1: player1, 2: player2

    def reset(self, turn):
        if turn != 1 and turn != 2:
            self.turn = randint(1, 2)
        else:
            self.turn = turn
        self.moves = 0
        self.winner = -1
        self.tiles = [['r','h','e','a','g','a','e','h','r'],
                      [' ',' ',' ',' ',' ',' ',' ',' ',' '],
                      [' ','c',' ',' ',' ',' ',' ','c',' '],
                      ['s',' ','s',' ','s',' ','s',' ','s'],
                      [' ',' ',' ',' ',' ',' ',' ',' ',' '],
                      [' ',' ',' ',' ',' ',' ',' ',' ',' '],
                      ['S',' ','S',' ','S',' ','S',' ','S'],
                      [' ','C',' ',' ',' ',' ',' ','C',' '],
                      [' ',' ',' ',' ',' ',' ',' ',' ',' '],
                      ['R','H','E','A','G','A','E','H','R']]
        '''
        row up to down: 0 - 9
        column left to right: 0 - 8
        command input format: xyx'y' 
        '''
        print('Game start!\nPlayer1: uppercase letters; Player2: lowercase letters\nType "q" to quit, "r" to restart')

    def forfeit(self, player):
        self.winner = 3 - player

    def move(self, start, end):
        # check winning
        if self.tiles[end[1]][end[0]] == 'G':
            self.winner = 2
        elif self.tiles[end[1]][end[0]] == 'g':
            self.winner = 1

        # perform moving
        self.tiles[end[1]][end[0]] = self.tiles[start[1]][start[0]]
        self.tiles[start[1]][start[0]] = ' '

        # update turn
        if self.winner < 0 and (self.turn == 1 or self.turn == 2):
            self.turn = 3 - self.turn
        else:
            self.turn = 0

        self.moves += 1

    def check_legal(self, start, end):
        # check if coordinate in range
        if start[0] > W-1 or end[0] > W-1 or \
           (start[0] == end[0] and start[1] == end[1]):
           return False

        piece = self.tiles[start[1]][start[0]]
        # piece is null or not the player's
        if piece == ' ' or \
           (self.turn == 1 and piece.islower()) or \
           (self.turn == 2 and piece.isupper()):
            print('Invalid piece: ' + piece, self.turn)
            return False

        target = self.tiles[end[1]][end[0]]
        # target is player's own piece
        if target != ' ' and \
           (self.turn == 1 and target.isupper()) or \
           (self.turn == 2 and target.islower()):
            print('Invalid target: ' + target)
            return False

        # check again if start and end are the same
        delta = (end[0] - start[0], end[1] - start[1])
        if delta == (0, 0):
            print('No move')
            return False

        # piece specific rules
        # Chariot (R)
        if piece == 'R' or piece == 'r':
            if delta[0] != 0 and delta[1] != 0:
                return False

            if delta[0] > 0:
                for i in range(1, delta[0]):
                    if self.tiles[start[1]][start[0]+i] != ' ':
                        return False
            elif delta[0] < 0:
                for i in range(1, -delta[0]):
                    if self.tiles[start[1]][start[0]-i] != ' ':
                        return False
            elif delta[1] > 0:
                for i in range(1, delta[1]):
                    if self.tiles[start[1]+i][start[0]] != ' ':
                        return False
            elif delta[1] < 0:
                for i in range(1, -delta[1]):
                    if self.tiles[start[1]-i][start[0]] != ' ':
                        return False
            return True

        # Horse (H)
        elif piece == 'H' or piece == 'h':
            if delta not in {(-1, -2), (-2, -1), (1, -2), (2, -1), \
                             (-1, 2), (-2, 1), (1, 2), (2, 1)}:
                return False

            if (delta[0] == 2 and self.tiles[start[1]][start[0]+1]) or \
               (delta[0] == -2 and self.tiles[start[1]][start[0]-1]) or \
               (delta[1] == 2 and self.tiles[start[1]+1][start[0]]) or \
               (delta[0] == -2 and self.tiles[start[1]-1][start[0]]):
                return False

            return True

        # Elephant (E)
        elif piece == 'E' or piece == 'e':
            if delta not in {(-2, -2), (-2, 2), (2, -2), (2, 2)}:
                return False

            if (delta == (2, 2) and self.tiles[start[1]+1][start[0]+1]) or \
               (delta == (-2, 2) and self.tiles[start[1]+1][start[0]-1]) or \
               (delta == (2, -2) and self.tiles[start[1]-1][start[0]+1]) or \
               (delta == (-2, -2) and self.tiles[start[1]-1][start[0]-1]):
                return False

            if (piece == 'e' and end[1] > 4) or (piece == 'E' and end[1] < 5):
                return False

            return True

        # Assistant (A)
        elif piece == 'A' or piece == 'a':
            if delta not in {(-1, -1), (-1, 1), (1, -1), (1, 1)}:
                return False

            if end[0] < 3 or end[0] > 5 or (end[1] > 2 and end[1] < 7):
                return False

            return True

        # General (G)
        elif piece == 'G' or piece == 'g':
            # eating the opponent's general
            if (target == 'g' or target == 'G') and delta[0] == 0:
                if delta[1] > 0:
                    for i in range(1, delta[1]):
                        if self.tiles[start[1]+i][start[0]] != ' ':
                            return False
                elif delta[1] < 0:
                    for i in range(1, -delta[1]):
                        if self.tiles[start[1]-i][start[0]] != ' ':
                            return False

            elif delta not in {(1, 0), (-1, 0), (0, -1), (0, 1)}:
                return False

            if end[0] < 3 or end[0] > 5 or (end[1] > 2 and end[1] < 7):
                return False

            return True

        # Cannon (C)
        elif piece == 'C' or piece == 'c':
            if delta[0] != 0 and delta[1] != 0:
                return False

            count = 0
            if delta[0] > 0:
                for i in range(1, delta[0]):
                    if self.tiles[start[1]][start[0]+i] != ' ':
                        count += 1
            elif delta[0] < 0:
                for i in range(1, -delta[0]):
                    if self.tiles[start[1]][start[0]-i] != ' ':
                        count += 1
            elif delta[1] > 0:
                for i in range(1, delta[1]):
                    if self.tiles[start[1]+i][start[0]] != ' ':
                        count += 1
            elif delta[1] < 0:
                for i in range(1, -delta[1]):
                    if self.tiles[start[1]-i][start[0]] != ' ':
                        count += 1

            if target == ' ':
                return count == 0
            else:
                return count == 1

        # Soldier (S)
        elif piece == 'S' or piece == 's':
            if piece == 'S':
                if start[1] > 4:
                    return delta == (0, -1)
                else:
                    return (delta in {(0, -1), (-1, 0), (1, 0)})
            else:
                if start[1] < 5:
                    return delta == (0, 1)
                else:
                    return (delta in {(0, 1), (-1, 0), (1, 0)})

        else:
            return False

    def check_legal(self, cmd):
        start = (int(cmd[0]), int(cmd[1]))
        end = (int(cmd[2]), int(cmd[3]))
        return self.check_legal(start, end)

    def get_winner(self):
        return self.winner

    def get_turn(self):
        return self.turn

    def log_info(self):
        print("Player{}'s turn.".format(self.turn))

    def print_board(self):
        for row in self.tiles:
            print(row)

def main():
    board = Board()
    board.reset(1)
    board.print_board()
    while(True):
        board.log_info()
        cmd = str(input("Please type command (xyx'y'): "))
        if not cmd.isdecimal or len(cmd) != 4:
            if cmd == 'q':
                print("Quiting game")
                break
            elif cmd == 'r':
                print("Restarting game")
                board.reset(0)
                board.print_board()
                continue
            else:
                print("Invalid command!")
                continue

        start = (int(cmd[0]), int(cmd[1]))
        end = (int(cmd[2]), int(cmd[3]))

        if start[0] > W-1 or end[0] > W-1 or \
           (start[0] == end[0] and start[1] == end[1]):
            print("Invalid command!")
            continue

        if not board.check_legal(start, end):
            print("Illegal move")
            continue

        board.move(start, end)
        board.print_board()


if __name__ == "__main__":
    main()
