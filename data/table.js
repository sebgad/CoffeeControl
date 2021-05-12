google.charts.load('current', {'packages':['table']});
google.charts.setOnLoadCallback(drawTable);

function drawTable() {
    var data = new google.visualization.DataTable();
    data.addColumn('string', 'Name');
    data.addColumn('boolean', 'Status');
    data.addRows([
    ['Pump', true],
    ['Heating',  false],
    ['Main-Switch', true],
    ]);
    // check each row
    for (var i = 0; i < data.getNumberOfRows(); i++) {
        // check boolean value
        if (data.getValue(i, 1)) {
        data.setProperty(i, 1, 'style', 'background-color: orange; color:orange; width:5%');
    } else {
        data.setProperty(i, 1, 'style', 'background-color: grey; color:grey; width:5%');
    }
    }

    var table = new google.visualization.Table(document.getElementById('table_div'));
        table.draw(data, {showRowNumber: false, width: '30%', height: '100%', allowHtml: true,});
}

