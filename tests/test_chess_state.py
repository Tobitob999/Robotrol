import unittest

from chess.state import board_to_fen


class TestChessState(unittest.TestCase):
    def test_board_to_fen(self):
        board = [
            list("rnbqkbnr"),
            list("pppppppp"),
            list("........"),
            list("........"),
            list("........"),
            list("........"),
            list("PPPPPPPP"),
            list("RNBQKBNR"),
        ]
        fen = board_to_fen(board, white_bottom=True)
        self.assertEqual(fen, "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR")


if __name__ == "__main__":
    unittest.main()
