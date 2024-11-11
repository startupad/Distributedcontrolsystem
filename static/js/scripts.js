// scripts.js

(function () {
  let matrix = [[]];

  function renderMatrix() {
    const container = document.getElementById("matrix-container");
    container.innerHTML = "";

    matrix.forEach((row, i) => {
      const rowDiv = document.createElement("div");
      row.forEach((cell, j) => {
        const input = document.createElement("input");
        input.type = "number";
        input.value = cell || "";
        input.onchange = (e) => {
          matrix[i][j] = parseFloat(e.target.value);
        };
        rowDiv.appendChild(input);
      });
      container.appendChild(rowDiv);
    });
  }

  function addRow() {
    const columns = matrix[0].length || 1;
    const newRow = new Array(columns).fill("");
    matrix.push(newRow);
    renderMatrix();
  }

  function addColumn() {
    matrix.forEach((row) => {
      row.push("");
    });
    renderMatrix();
  }

  function sendMatrix() {
    fetch("/submit_matrix", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ matrix: matrix }),
    })
      .then((response) => response.json())
      .then((data) => {
        alert("Matrice inviata con successo!");
        console.log("Success:", data);
        // Reset the matrix if needed
        matrix = [[]];
        renderMatrix();
      })
      .catch((error) => {
        console.error("Error:", error);
      });
  }

  // Initialize the matrix on window load
  window.onload = function () {
    renderMatrix();
  };
})();
