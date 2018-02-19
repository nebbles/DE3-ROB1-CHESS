"""
This engine is designed to interface with the modified sunfish file to
provide a specialised interface between the other modules in this project
and the chess logic underneath.
"""

import multiprocessing as mp
import time
import sys
try:
    import chess.sunfish_custom as sunfish
except ImportError:
    import sunfish_custom as sunfish



class HiddenPrints:
    def __enter__(self):
        self._original_stdout = sys.stdout
        sys.stdout = None

    def __exit__(self, exc_type, exc_val, exc_tb):
        sys.stdout = self._original_stdout


class Error(Exception):
    """Base class for exceptions in this module."""
    pass


class EngineError(Error):
    """Exception raised for errors in the game engine.

    Attributes:
        message -- explanation of the error
    """

    def __init__(self, message):
        self.message = message


class ChessState:
    """Class holding the ongoing state of the chess board."""
    def __init__(self, debug=False):
        self.debug = debug
        self.board = list(
            'rnbqkbnr'  # 00 - 07
            'pppppppp'  # 08 - 15
            '........'  # 16 - 23
            '........'  # 24 - 31
            '........'  # 32 - 39
            '........'  # 40 - 47
            'PPPPPPPP'  # 48 - 55
            'RNBQKBNR'  # 56 - 63
        )
        self.graveyard = []
        self.letters = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']

        self.move_from_index = None
        self.move_to_index = None

        self.move_from_pos = None
        self.move_to_pos = None
        self.move_from_piece = None

        self.user_move = None

        self.col_labels = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
        self.row_labels = ['8', '7', '6', '5', '4', '3', '2', '1']

    def add_to_graveyard(self, piece):
        self.graveyard.append(piece)

    def get_bwe(self):
        bwe = []
        for i in range(len(self.board)):
            if self.board[i] == '.':
                bwe.append('E')
            elif self.board[i].islower():
                bwe.append('B')
            elif self.board[i].isupper():
                bwe.append('W')
            else:
                raise EngineError("Unsupported board piece found")
        return bwe

    def get_bwe_move(self, bwe):
        """Checks BWE passed in to see if valid move was made. Returns boolean."""
        move = self.compare_bwe(bwe)  # compare BWE
        piece = self.board[move[0]]  # get piece type
        if self.debug:
            print("piece: ", piece)
            print("start pos", move[0])

        move_from = self.convert_to_pos(move[0])
        move_to = self.convert_to_pos(move[1])

        return piece, move_from+move_to

    def compare_bwe(self, new_bwe):
        """Take a new BWE list. Compare to the existing state of the game. Return tuple of ``(
        move_from, move_to)`` indices"""
        m1 = self.get_bwe()
        m2 = new_bwe

        if len(m1) != len(m2):
            raise EngineError("Two unequally sized BWE matrices cannot be compared")

        mismatches = []
        for i in range(len(m1)):
            if m1[i] != m2[i]:
                mismatches.append(i)

        if len(mismatches) > 2:
            raise EngineError("More than two positions on the board have changed!")

        index1 = mismatches[0]
        index2 = mismatches[1]
        if self.board[index1] == '.' or self.board[index1].islower():
            return index2, index1
        else:
            return index1, index2

    def convert_to_pos(self, index_num):

        row = index_num // 8
        col = index_num - 8 * (index_num // 8)

        coord = self.col_labels[col] + self.row_labels[row]

        return coord

    def convert_to_index(self, chess_pos):
        col_label = chess_pos[0]
        row_label = chess_pos[1]

        col = self.col_labels.index(col_label)
        row = self.row_labels.index(row_label)

        return row * 8 + col

    def update_board(self, bwe_matrix):
        """
        This updates the game state with the latest BWE matrix after the use
        plays their turn.
        """
        move = self.compare_bwe(bwe_matrix)  # Start and end locations of moving piece
        self.move_from_index = move[0]
        self.move_to_index = move[1]

        # If there was a black piece, add b piece to graveyard
        if self.board[move[1]].islower():
            self.add_to_graveyard(self.board[move[1]])

        # Convert all position indices into chess locations
        self.move_from_pos = self.convert_to_pos(self.move_from_index)
        self.move_to_pos = self.convert_to_pos(self.move_to_index)
        self.move_from_piece = self.board[self.move_from_index]
        self.user_move = self.move_from_pos + self.move_to_pos

        if self.debug:
            print("Piece moved by user: ", self.move_from_piece)
            print("Moved from: ", self.move_from_pos)
            print("Moved to: ", self.move_to_pos)
            print("Pieces in the graveyard: ", self.graveyard)

        # Update internal board
        self.board[self.move_to_index] = self.board[self.move_from_index]
        self.board[self.move_from_index] = '.'
        if self.debug:
            print(self.board)


class ChessEngine:
    def __init__(self, debug=False, suppress_sunfish=True):
        self.debug = debug
        self.suppress_sunfish = suppress_sunfish
        self.state = ChessState(debug=False)

        self.command_q = mp.Queue()
        self.reply_q = mp.Queue()
        self.valid_q = mp.Queue()
        if self.debug:
            print("Starting Sunfish...")

        if suppress_sunfish:
            with HiddenPrints():
                self.start_sunfish_process()
        else:
            self.start_sunfish_process()

    def start_sunfish_process(self):
        chess_ai = mp.Process(target=sunfish.main, args=(self.command_q, self.reply_q,
                                                         self.valid_q))
        chess_ai.daemon = True
        chess_ai.start()

    def input_bwe(self, bwe):
        """Takes in the latest BWE and tries to input that to Sunfish AI.

        If it succeeds, it updates the internal game state and returns 1, and command list.
        If it fails, it returns a 0, and what move it thinks failed."""

        if self.debug:
            print("Comparing the BWE matrix to game state")
        move = self.state.get_bwe_move(bwe)  # get the move from the bwe matrix
        if self.debug:
            print("Passing the move to the Sunfish AI: ", move)

        self.command_q.put(move[1])  # pass the move to the chess ai

        # TODO: check if user move caused checkmate
        valid = self.valid_q.get(block=True)

        if self.debug:
            print("Validity from Sunfish AI: ", valid)

        if not valid:
            return 0, move  # move was invalid, return 0 and invalid move
        elif valid == 1:
            self.state.update_board(bwe)  # update the board with the latest move
            reply = self.reply_q.get(block=True)  # get the return move from Sunfish
            if self.debug:
                print("Sunfish replied with: ", reply)

            # split the reply
            move_from_pos = reply[0:2]
            move_to_pos = reply[2:]
            move_from_index = self.state.convert_to_index(move_from_pos)
            move_to_index = self.state.convert_to_index(move_to_pos)
            move_from_piece = self.state.board[move_from_index]

            response = []

            if self.state.board[move_to_index].isupper():
                kill_piece = self.state.board[move_to_index]
                response.append((kill_piece, move_to_pos))
            else:
                kill_piece = None

            response.append((move_from_piece, move_from_pos+move_to_pos))

            if self.debug:
                print("Computer move from: ", move_from_pos)
                print("to: ", move_to_pos)
                print("with piece: ", move_from_piece)
                print("killing this piece: ", kill_piece)

            return 1, response

    def test(self):
        """Test is only used when debugging and developing the engine. It should not be called by
        any other element of the program or from other modules."""

        # EXAMPLE CHANGES FROM A USER
        test2 = ChessState()
        test2.board[48] = '.'
        test2.board[40] = 'P'  # or 40 / 8
        new_bwe = test2.get_bwe()
        # print("BWE: ", new_bwe)

        # TODO: check BWE first for illegal moves
        # result = self.state.compare_bwe(new_bwe)
        # print(result)
        success, response = self.input_bwe(new_bwe)
        print(success, response)

        sys.exit()
        self.state.update_board(new_bwe)

        # Tell sunfish this move
        move_msg = self.state.user_move
        print("Tell sunfish: ", move_msg)
    
        if True:
            time.sleep(3)
            print("putting in a2a3")
            self.command_q.put('a2a3')

            validity = self.valid_q.get(block=True)
            print("VALIDITY: ", validity)

            while self.reply_q.empty():
                time.sleep(1)
                print("reply q is empty")
            print("I JUST GOT: ", self.reply_q.get())
    
            time.sleep(3)
            print("putting in a3a4")
            self.command_q.put('a3a7')

            validity = self.valid_q.get(block=True)
            print("VALIDITY 2: ", validity)
    
            time.sleep(3)
            sys.exit()

        if False:
            reply = 'h7h2'  # debug value
            print("sunfish reply: ", reply)
    
        # TODO: check if sunfish reply caused checkmate
        # split the reply
        move_from_pos = reply[0:2]
        move_to_pos = reply[2:]
    
        move_from_index = self.state.convert_to_index(move_from_pos)
        move_to_index = self.state.convert_to_index(move_to_pos)
    
        move_from_piece = self.state.board[move_from_index]
    
        if self.state.board[move_to_index].isupper():
            kill_piece = self.state.board[move_to_index]
        else:
            kill_piece = None
    
        print("Computer move from: ", move_from_pos)
        print("to: ", move_to_pos)
        print("with piece: ", move_from_piece)
        print("killing this piece: ", kill_piece)
    
        # return the move information to the caller


if __name__ == '__main__':

    engine = ChessEngine(debug=True, suppress_sunfish=False)
    engine.test()
