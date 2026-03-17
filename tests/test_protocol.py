"""Test the serial protocol parser."""

import pytest

from robotrol.serial.protocol import parse_status_line, is_homing_command


class TestParseStatusLine:
    """Tests for parse_status_line()."""

    def test_idle_with_mpos(self):
        result = parse_status_line("<Idle|MPos:1.0,2.0,3.0,4.0,5.0,6.0>")
        assert result["state"] == "Idle"
        assert result["mpos"]["X"] == 1.0
        assert result["mpos"]["Y"] == 2.0
        assert result["mpos"]["Z"] == 3.0
        assert result["mpos"]["A"] == 4.0
        assert result["mpos"]["B"] == 5.0
        assert result["mpos"]["C"] == 6.0

    def test_run_with_mpos_and_wco(self):
        result = parse_status_line("<Run|MPos:0,0,0,0,0,0|WCO:1,2,3,4,5,6>")
        assert result["state"] == "Run"
        for ax in ["X", "Y", "Z", "A", "B", "C"]:
            assert result["mpos"][ax] == 0.0
        assert result["wco"]["X"] == 1.0
        assert result["wco"]["Y"] == 2.0
        assert result["wco"]["Z"] == 3.0
        assert result["wco"]["A"] == 4.0
        assert result["wco"]["B"] == 5.0
        assert result["wco"]["C"] == 6.0

    def test_invalid_line_returns_none_state(self):
        result = parse_status_line("not a status line")
        assert result["state"] is None
        assert result["mpos"] == {}

    def test_alarm_state(self):
        result = parse_status_line("<Alarm|MPos:0,0,0,0,0,0>")
        assert result["state"] == "Alarm"

    def test_with_pn_field(self):
        result = parse_status_line("<Idle|MPos:0,0,0,0,0,0|Pn:XZ>")
        assert "X" in result["pn"]
        assert "Z" in result["pn"]

    def test_with_fs_field(self):
        result = parse_status_line("<Idle|MPos:0,0,0,0,0,0|FS:1000,500>")
        assert result["fs"] == (1000, 500)


class TestIsHomingCommand:
    """Tests for is_homing_command()."""

    def test_dollar_h(self):
        assert is_homing_command("$H") is True

    def test_dollar_hx(self):
        assert is_homing_command("$HX") is True

    def test_g1_not_homing(self):
        assert is_homing_command("G1 X10") is False

    def test_dollar_x_not_homing(self):
        assert is_homing_command("$X") is False

    def test_empty_string(self):
        assert is_homing_command("") is False

    def test_none_input(self):
        assert is_homing_command(None) is False

    def test_with_whitespace(self):
        assert is_homing_command("  $H  ") is True
