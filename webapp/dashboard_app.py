"""
Replay dashboard for the IMU Sensor-Fusion Node.

Unlike Project 13's climate monitor (which reports over MQTT to a cloud
broker, so a hosted web dashboard can subscribe to it live from anywhere),
this project's device talks over a WIRED USB serial connection straight
to whatever computer it's plugged into -- there's no cloud broker in the
loop, and a hosted Streamlit app has no way to reach a COM port on your
own machine. Being honest about that boundary matters more than faking
a "live" demo that couldn't actually work: this dashboard is a REPLAY and
analysis tool instead. It reads the CSV the real C++ host app's
--csv-out writes (recorded from a real live session, or from --replay
against a captured/synthetic file) and charts it -- and it can generate
its own synthetic sample session on the spot, so a visitor with no
hardware and no recorded file can still see the whole system's output.

Run locally:
    streamlit run dashboard_app.py
"""

import sys
from pathlib import Path

import pandas as pd
import streamlit as st

SIM_PATH = Path(__file__).resolve().parent.parent / "sim"
sys.path.insert(0, str(SIM_PATH))

from imu_sim.fusion import KalmanFilter1D, ComplementaryFilter, accel_to_pitch  # noqa: E402
from imu_sim.synth import generate_motion  # noqa: E402

st.set_page_config(page_title="IMU Sensor-Fusion Node — Replay Dashboard", layout="wide")

REQUIRED_COLUMNS = {"seq", "device_us", "accel_x", "accel_y", "accel_z",
                    "gyro_x", "gyro_y", "gyro_z", "fused_pitch_deg"}


def generate_sample_session(seed: int, use_kalman: bool) -> pd.DataFrame:
    """Builds a session the same shape as a real --csv-out file, using the
    same imu_sim package every notebook and lab in this project uses --
    the dashboard's 'no file needed' path is real project code, not a
    separate mock."""
    rows = generate_motion(duration_s=20.0, sample_rate_hz=200.0, seed=seed)
    dt = 1.0 / 200.0
    fuser = KalmanFilter1D() if use_kalman else ComplementaryFilter()

    records = []
    for i, row in enumerate(rows):
        accel_angle = accel_to_pitch(row["accel_y"], row["accel_z"], row["accel_x"])
        fused = fuser.update(accel_angle, row["gyro_y"], dt)
        records.append({
            "seq": i, "device_us": row["device_us"],
            "accel_x": row["accel_x"], "accel_y": row["accel_y"], "accel_z": row["accel_z"],
            "gyro_x": row["gyro_x"], "gyro_y": row["gyro_y"], "gyro_z": row["gyro_z"],
            "fused_pitch_deg": fused,
        })
    return pd.DataFrame(records)


def main():
    st.title("IMU Sensor-Fusion Node")
    st.caption("A replay/analysis dashboard — no live cloud connection, on purpose. "
               "See the note in the sidebar for why.")

    with st.sidebar:
        st.header("Load a session")
        uploaded = st.file_uploader("Upload a --csv-out file from the real host app", type="csv")
        st.divider()
        st.subheader("...or generate a synthetic one")
        seed = st.number_input("Random seed", min_value=0, max_value=9999, value=42)
        use_kalman = st.checkbox("Use Kalman filter (unchecked = complementary filter)", value=True)
        generate_clicked = st.button("Generate synthetic session")
        st.divider()
        st.info(
            "Why no live mode? This device talks over a wired USB serial "
            "connection to whatever computer it's plugged into -- there's no "
            "cloud broker in the loop the way Project 13's MQTT-based climate "
            "monitor has, so a hosted dashboard has no way to reach a real COM "
            "port on your machine. Run the real host app locally "
            "(`../host/build/imu_host --port COM5 --csv-out session.csv`) and "
            "upload the result here, or use the synthetic generator above."
        )

    df = None
    if uploaded is not None:
        df = pd.read_csv(uploaded)
        missing = REQUIRED_COLUMNS - set(df.columns)
        if missing:
            st.error(f"This CSV is missing expected columns: {sorted(missing)}")
            df = None
    elif generate_clicked or "session_df" in st.session_state:
        if generate_clicked:
            st.session_state["session_df"] = generate_sample_session(seed, use_kalman)
        df = st.session_state["session_df"]

    if df is None:
        st.info("Upload a real session CSV, or click **Generate synthetic session** in the sidebar.")
        return

    df["t_s"] = df["device_us"] / 1_000_000.0

    m1, m2, m3 = st.columns(3)
    m1.metric("Frames", len(df))
    m2.metric("Duration", f"{df['t_s'].max() - df['t_s'].min():.1f} s")
    m3.metric("Final fused pitch", f"{df['fused_pitch_deg'].iloc[-1]:.1f} deg")

    st.subheader("Fused pitch over time")
    st.line_chart(df.set_index("t_s")[["fused_pitch_deg"]])

    st.subheader("Raw sensor axes")
    accel_col, gyro_col = st.columns(2)
    accel_col.line_chart(df.set_index("t_s")[["accel_x", "accel_y", "accel_z"]])
    gyro_col.line_chart(df.set_index("t_s")[["gyro_x", "gyro_y", "gyro_z"]])

    st.subheader("Raw data")
    st.dataframe(df, width="stretch", hide_index=True)


if __name__ == "__main__":
    main()
