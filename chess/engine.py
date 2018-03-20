"""
This engine is designed to interface with the modified sunfish file to
provide a specialised interface between the other modules in this project
and the chess logic underneath.
"""
from __future__ import print_function
import multiprocessing as mp
import time
import sys
import chess.sunfish_custom as sunfish


class HiddenPrints:
    """Context manager for suppressing the print output of functions within its scope."""
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
        self.letters = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']

        self.move_from_index = None
        self.move_to_index = None

        self.move_from_pos = None
        self.move_to_pos = None
        self.move_from_piece = None

        self.user_move = None

        self.col_labels = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
        self.row_labels = ['8', '7', '6', '5', '4', '3', '2', '1']

    def get_bwe(self):
        """Returns the current game state as a BWE list."""
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
        """Takes a BWE list and returns the move that was made.

        Attributes:
            * bwe: A list of single character strings either 'B','W', or 'E'.

        Returns:
            * piece (str): Type of piece that moved e.g. ``'P'``
            * move (str): Move recognised in BWE e.g. ``'a2a4'``
        """
        move = self.compare_bwe(bwe)  # compare BWE
        piece = self.board[move[0]]  # get piece type
        if self.debug:
            print("piece: ", piece)
            print("start pos", move[0])

        move_from = self.convert_to_pos(move[0])
        move_to = self.convert_to_pos(move[1])

        return piece, move_from+move_to

    def compare_bwe(self, new_bwe):
        """Takes a BWE list and compares it to the existing game state. Return tuple of
        ``(move_from, move_to)`` indices for the single move that's detected. Does not verify if
        move is a legal one."""
        m1 = self.get_bwe()
        m2 = new_bwe

        b_before = m1.count('B')
        w_before = m1.count('W')
        e_before = m1.count('E')
        b_after = m2.count('B')
        w_after = m2.count('W')
        e_after = m2.count('E')

        if self.debug:
            print("Number of B elements before: ", b_before)
            print("Number of W elements before: ", w_before)
            print("Number of E elements before: ", e_before)
            print("Number of B elements after: ", b_after)
            print("Number of W elements after: ", w_after)
            print("Number of E elements after: ", e_after)

        # Conduct sanity checks
        if len(m1) != len(m2):
            raise EngineError("Two unequally sized BWE matrices cannot be compared")
        if not b_before-1 <= m2.count('B') <= 16:
            raise EngineError("Invalid number of black pieces in new BWE")
        if not w_before <= m2.count('W') <= 16:
            raise EngineError("Invalid number of white pieces in new BWE")
        max_empties = 64 - b_before - w_before + 1
        if not 32 <= m2.count('E') <= max_empties:
            raise EngineError("Invalid number of empty squares in new BWE")

        mismatches = []
        for i in range(len(m1)):
            if m1[i] != m2[i]:
                mismatches.append(i)

        if len(mismatches) > 2:
            raise EngineError("More than two positions on the board have changed!")

        index1 = mismatches[0]
        index2 = mismatches[1]

        piece1 = self.board[index1]
        piece2 = self.board[index2]

        if self.debug:
            print("Mismatch 1: ", piece1, index1)
            print("Mismatch 2: ", piece2, index2)

        if piece1 == '.' or piece1.islower():
            return index2, index1
        elif piece2 == '.' or piece2.islower():
            return index1, index2
        else:
            raise EngineError("There was a problem detecting the move in the BWE")

    def convert_to_pos(self, index_num):
        """Takes an board index (0-63) and converts to the corresponding board position (e.g.
        'a2')."""

        row = index_num // 8
        col = index_num - 8 * (index_num // 8)

        coord = self.col_labels[col] + self.row_labels[row]

        return coord

    def convert_to_index(self, chess_pos):
        """Takes an board position (e.g. 'a2') and converts to the corresponding board index (
        0-63)."""
        col_label = chess_pos[0]
        row_label = chess_pos[1]

        col = self.col_labels.index(col_label)
        row = self.row_labels.index(row_label)

        return row * 8 + col

    def update_board(self, bwe_matrix):
        """Updates the game state with the latest BWE matrix after the user has played their
        turn. The BWE has been checked and the move has been checked with Sunfish.
        """
        move = self.compare_bwe(bwe_matrix)  # Start and end locations of moving piece
        self.move_from_index = move[0]
        self.move_to_index = move[1]

        # Convert all position indices into chess locations
        self.move_from_pos = self.convert_to_pos(self.move_from_index)
        self.move_to_pos = self.convert_to_pos(self.move_to_index)
        self.move_from_piece = self.board[self.move_from_index]
        self.user_move = self.move_from_pos + self.move_to_pos

        if self.debug:
            print("Piece moved by user: ", self.move_from_piece)
            print("Moved from: ", self.move_from_pos)
            print("Moved to: ", self.move_to_pos)

        # Update internal board
        self.board[self.move_to_index] = self.board[self.move_from_index]
        self.board[self.move_from_index] = '.'
        if self.debug:
            print(self.board)


class ChessEngine:
    """Engine that manages communication between main program and chess AI Sunfish (running in
    separate process).

    It's main purpose is to take a BWE matrix as the user's potential move and provide an
    analysis of this move by either reporting back its invalidity or the AI's response.
    """
    def __init__(self, debug=False, suppress_sunfish=True):
        self.debug = debug
        self.suppress_sunfish = suppress_sunfish
        self.state = ChessState(debug=self.debug)

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
        """Spins up external process for Sunfish AI.

        Process communicates with three queues, the command queue (for user moves), the valid queue
        for confirming if the user queue is valid, and the reply queue for the Sunfish computer
        move response.
        """
        chess_ai = mp.Process(target=sunfish.main, args=(self.command_q, self.reply_q,
                                                         self.valid_q))
        chess_ai.daemon = True
        chess_ai.start()

    def input_bwe(self, bwe):
        """Takes in the latest BWE and tries to input that to Sunfish AI.

        Returns:
            code : int
                * ``-1``, BWE was invalid
                * ``0``, move was invalid
                * ``1``, user won the game with move
                * ``2``, computer won the game with checkmate
                * ``3``, computer did not win and

            result
                Depending on ``code``, this will vary:

                * ``-1``, None
                * ``0``, Move that was invalid as string
                * ``1``, None
                * ``2``, Move that computer is playing to win as tuple
                * ``3``, Move that computer is playing as tuple

        Computer move is a list of tuples in the form:

        * ``[ (piece_to_move, move) ]``, or
        * ``[ (piece_to_kill, location), (piece_to_move, move) ]``

        where,

        * Pieces are single character strings.
        * Locations are two character strings e.g. 'a2'
        * Moves are 4 character strings e.g. 'a2a4'
        """

        if self.debug:
            print("Comparing the BWE matrix to game state")
        try:
            move = self.state.get_bwe_move(bwe)  # get the move from the bwe matrix
        except EngineError:
            return -1, None
        if self.debug:
            print("Passing the move to the Sunfish AI: ", move)

        self.command_q.put(move[1])  # pass the move to the chess ai

        valid = self.valid_q.get(block=True)

        if self.debug:
            print("Validity from Sunfish AI: ", valid)

        if not valid:
            return 0, move  # move was invalid, return 0 and invalid move
        elif valid == 1:
            if self.debug:
                print("Sunfish has reported that user won with move: ", move)
            return 1
        elif valid in [2, 3]:
            if valid == 2:
                print("Validity was 2 so computer made checkmate")
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

            # First, construct response to return to caller
            response = []
            if self.state.board[move_to_index].isupper():
                kill_piece = self.state.board[move_to_index]
                response.append((kill_piece, move_to_pos))
            else:
                kill_piece = None
            response.append((move_from_piece, move_from_pos+move_to_pos))

            # Second, update the chess state with the reply for reference on next turn
            self.state.board[move_to_index] = self.state.board[move_from_index]
            self.state.board[move_from_index] = '.'

            if self.debug:
                print("Computer move from: ", move_from_pos)
                print("to: ", move_to_pos)
                print("with piece: ", move_from_piece)
                print("killing this piece: ", kill_piece)

            return valid, response

    def test(self):
        """This method is only used when debugging and developing the engine. It should
        not be called from other modules."""

        # EXAMPLE CHANGES FROM A USER
        test2 = ChessState()
        test2.board[48] = '.'
        test2.board[8] = 'P'  # or 40 / 8
        # test2.board[41] = 'P'
        # test2.board[40] = 'P'  # or 40 / 8

        new_bwe = test2.get_bwe()
        print("BWE: ", new_bwe)

        result = self.state.get_bwe_move(new_bwe)
        print(result)

        # TODO: check BWE first for illegal moves
        # result = self.state.compare_bwe(new_bwe)
        # print(result)
        success, response = self.input_bwe(new_bwe)
        print(success, response)

        return
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
