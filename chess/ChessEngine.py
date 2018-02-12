"""
This engine is designed to interface with the modified sunfish file to
provide a specialised interface between the other modules in this project
and the chess logic underneath.
"""

import multiprocessing as mp
import sunfishCustom as sunfish
import time
import sys


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
    def __init__(self):
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

    def compare_bwe(self, new_bwe):
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

        return mismatches

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
        mismatch_indices = self.compare_bwe(bwe_matrix)
        # Start and end locations of moving piece
        mm1 = mismatch_indices[0]
        mm2 = mismatch_indices[1]

        # Work out which index was start/end point
        if self.board[mm1] == '.' or self.board[mm1].islower():
            self.move_from_index = mm2
            self.move_to_index = mm1
        else:
            self.move_from_index = mm1
            self.move_to_index = mm2

        # If there was a black piece, add b piece to graveyard
        if self.board[mm1].islower():
            self.add_to_graveyard(self.board[mm1])
        elif self.board[mm2].islower():
            self.add_to_graveyard(self.board[mm2])

        # Convert all position indices into chess locations
        self.move_from_pos = self.convert_to_pos(self.move_from_index)
        self.move_to_pos = self.convert_to_pos(self.move_to_index)
        self.move_from_piece = self.board[self.move_from_index]
        self.user_move = self.move_from_pos + self.move_to_pos

        print("Piece moved by user: ", self.move_from_piece)
        print("Moved from: ", self.move_from_pos)
        print("Moved to: ", self.move_to_pos)
        print("Pieces in the graveyard: ", self.graveyard)

        # Update internal board
        self.board[self.move_to_index] = self.board[self.move_from_index]
        self.board[self.move_from_index] = '.'
        print(self.board)


def chess_engine(sunfish):

    test1 = ChessState()

    # EXAMPLE CHANGES FROM A USER
    test2 = ChessState()
    test2.board[48] = '.'
    test2.board[8] = 'P'
    new_bwe = test2.get_bwe()


    # TODO ------- CONVERT TO CLASS

    test1.update_board(new_bwe)

    # Tell sunfish this move
    move_msg = test1.user_move
    print("Tell sunfish: ", move_msg)

    if False:  # TODO get computers move from sunfish
        command_q = mp.Queue()
        reply_q = mp.Queue()

        with HiddenPrints():
            sunfish = mp.Process(target=sunfish.main, args=(command_q, reply_q))
            sunfish.daemon = True
            sunfish.start()

        time.sleep(3)
        print("putting in a2a3")
        command_q.put('a2a3')

        while reply_q.empty():
            time.sleep(0.05)
        print("I JUST GOT: ", reply_q.get())

        time.sleep(3)
        print("putting in a3a4")
        command_q.put('a3a4')
        # print(command_q.qsize())

        time.sleep(3)

    # TODO act on the sunfish reply
    reply = 'h7h2'  # debug value
    print("sunfish reply: ", reply)

    # split the reply
    move_from_pos = reply[0:2]
    move_to_pos = reply[2:]

    move_from_index = test1.convert_to_index(move_from_pos)
    move_to_index = test1.convert_to_index(move_to_pos)

    move_from_piece = test1.board[move_from_index]

    if test1.board[move_to_index].isupper():
        kill_piece = test1.board[move_to_index]
    else:
        kill_piece = None

    print("Computer move from: ", move_from_pos)
    print("to: ", move_to_pos)
    print("with piece: ", move_from_piece)
    print("killing this piece: ", kill_piece)

    # where does it move to, does it kill an piece, what piece
    # where does it move from, with what piece

    # done above


if __name__ == '__main__':
    chess_engine(sunfish)
