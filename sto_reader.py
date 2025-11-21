#!/usr/bin/env python3
"""
STO File Reader - Extract hip exo rotation data from OpenSim STO files.
Extracts: hip_exo_r_rotation.input, hip_exo_l_rotation.input,
          hip_exo_r_rotation, hip_exo_l_rotation
"""

import numpy as np
import os
import glob


def read_sto_file(filepath: str) -> dict:
    """
    Read an OpenSim STO file and extract hip exo rotation columns.

    Args:
        filepath: Path to the .sto file

    Returns:
        Dictionary with arrays for each column and metadata
    """
    # Target columns to extract
    target_columns = [
        'time',
        'hip_exo_r_rotation.input',
        'hip_exo_l_rotation.input',
        'hip_exo_r_rotation',
        'hip_exo_l_rotation'
    ]

    with open(filepath, 'r') as f:
        lines = f.readlines()

    # Parse header
    header_end = 0
    metadata = {}
    for i, line in enumerate(lines):
        line = line.strip()
        if '=' in line:
            key, value = line.split('=', 1)
            metadata[key] = value
        if line == 'endheader':
            header_end = i + 1
            break

    # Get column names (first line after endheader)
    column_names = lines[header_end].strip().split('\t')

    # Find indices of target columns
    column_indices = {}
    for col in target_columns:
        if col in column_names:
            column_indices[col] = column_names.index(col)
        else:
            print(f"Warning: Column '{col}' not found in file")

    # Parse data rows
    data_lines = lines[header_end + 1:]
    n_rows = len(data_lines)

    # Initialize arrays
    result = {col: np.zeros(n_rows) for col in column_indices.keys()}

    # Parse each row
    for row_idx, line in enumerate(data_lines):
        if not line.strip():
            continue
        values = line.strip().split('\t')
        for col, col_idx in column_indices.items():
            try:
                result[col][row_idx] = float(values[col_idx])
            except (IndexError, ValueError):
                result[col][row_idx] = np.nan

    # Add metadata
    result['metadata'] = metadata
    result['source_file'] = os.path.basename(filepath)

    return result


def save_extracted_data(data: dict, output_path: str):
    """
    Save extracted data to NPZ file.

    Args:
        data: Dictionary from read_sto_file()
        output_path: Path for output .npz file
    """
    # Separate arrays from metadata
    arrays = {k: v for k, v in data.items() if isinstance(v, np.ndarray)}

    # Save as compressed NPZ
    np.savez_compressed(output_path, **arrays)
    print(f"Saved to: {output_path}")
    print(f"  Columns: {list(arrays.keys())}")
    print(f"  Rows: {len(arrays.get('time', []))}")


def process_sto_file(sto_path: str, output_dir: str = None):
    """
    Process a single STO file and save extracted data.

    Args:
        sto_path: Path to input .sto file
        output_dir: Optional output directory (default: same as input)
    """
    if output_dir is None:
        output_dir = os.path.dirname(sto_path)

    # Read and extract
    data = read_sto_file(sto_path)

    # Generate output filename
    base_name = os.path.splitext(os.path.basename(sto_path))[0]
    output_path = os.path.join(output_dir, f"{base_name}_hip_exo.npz")

    # Save
    save_extracted_data(data, output_path)

    return data


def process_directory(input_dir: str, output_dir: str = None):
    """
    Process all STO files in a directory.

    Args:
        input_dir: Directory containing .sto files
        output_dir: Optional output directory
    """
    sto_files = glob.glob(os.path.join(input_dir, "*.sto"))
    print(f"Found {len(sto_files)} STO files")

    for sto_path in sorted(sto_files):
        print(f"\nProcessing: {os.path.basename(sto_path)}")
        process_sto_file(sto_path, output_dir)


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        # Default: process example file
        example_file = "test/offline_dataset/00001_-658.338.sto"
        if os.path.exists(example_file):
            data = process_sto_file(example_file)

            # Print sample data
            print("\nSample data (first 5 rows):")
            print("-" * 80)
            for col in ['time', 'hip_exo_r_rotation.input', 'hip_exo_l_rotation.input',
                       'hip_exo_r_rotation', 'hip_exo_l_rotation']:
                if col in data:
                    print(f"{col}: {data[col][:5]}")
        else:
            print("Usage: python sto_reader.py <sto_file_or_directory>")
    else:
        path = sys.argv[1]
        if os.path.isdir(path):
            output_dir = sys.argv[2] if len(sys.argv) > 2 else None
            process_directory(path, output_dir)
        elif os.path.isfile(path):
            output_dir = sys.argv[2] if len(sys.argv) > 2 else None
            process_sto_file(path, output_dir)
        else:
            print(f"Error: Path not found: {path}")
